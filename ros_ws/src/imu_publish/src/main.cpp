#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <imu_publish/com.h>
#include <stdio.h>
#include <string.h>
#include <imu_publish/atom_macro.h>
#include <imu_publish/atomprotocol.h>
#include <imu_publish/main.h>
#include <unistd.h> 
#include <time.h>
#include <stdlib.h>

using namespace std;

#define COMPORT_COMMUNICATION
#define DMP_OUTPUT

#define THREADCOUNT 1
//#define LINEAR_ACC

SaberData_HandleType saberDataHandle;

unsigned short usLength = 0, usRxLength = 0;
long long gValidCount = 0;

int resendCounter;
unsigned char * pBufferStart = NULL, *pCustomer = NULL;
unsigned char *pProducer = NULL, *pCurrent = NULL, *pBufferEnd = NULL;
unsigned char * p;
unsigned char chr[RING_BUFFER_SIZE * 2];
unsigned char chrBuffer[RING_BUFFER_SIZE];
u8 ringBuf[RING_BUFFER_SIZE];
u8 tempbuf[RING_BUFFER_SIZE] = { '\0' };
extern unsigned short CollectUARTData(char chrUARTBufferOutput[]);
unsigned char sum = 0;
char ucComNo[2] = { 0, 0 };

u32 unProcessBytes, previous_bytes, processedBytes;

int count_L;
int points = 0;
u8 receiveAck;

extern int dataReady;
extern int Receive_count;
extern void Process();
extern void DataPacketParser(u8 *pBuffer, u16 dataLen);
int timeout_count = 0;
u8 reTx_flag = 0;
u8 errorCode_ack;
u8 rcid;
u8 rmid;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_publisher");
	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/imu/data", 20);
	
	FILE *fp;
	u8 ret = 0;
	pBufferStart = (unsigned char *)ringBuf;
	pBufferEnd = pBufferStart + RING_BUFFER_SIZE;
	pCustomer = pBufferStart;
	pProducer = pBufferStart;


	// Create COMthread

	ATR_Linux_OpenCom("/dev/imu_CP4", 115200);
	/****************************** Switch to config mode ********************************************/
	Atom_switchModeReq(CONFIG_MODE);
	
	
	/****************************** Select packet***************************************************/
	// Enable packets,  use SET_BIT to enable packet and use CLEAR_BIT to disable packet.
	int bitEnable = 0;
	
	SET_BIT(bitEnable, PACKET_CAL_GYRO);
	SET_BIT(bitEnable, PACKET_QUAT_DATA);
	//linear_acc only supports "C3G" Seri
//#ifdef LINEAR_ACC
//	SET_BIT(bitEnable, PACKET_LINEAR_ACC);
//#endif

	SelectPackets(bitEnable);

	/****************************** Switch To measure mode***************************************************************/
	Atom_switchModeReq(MEASURE_MODE);
	
	
	int seq = 0;
	int index = 0;
	int frameStart = 0;
	int ind = 0;
	unsigned char * pStart = NULL;
	int remainBytes = 0;
	unsigned char cid = 0x06;
	unsigned char mid = 0x81;

	signed char cResult[2] = { 0 };

	sensor_msgs::Imu imuMsg;
	struct timespec time_start={0,0}, time_end={0,0};

	ros::Rate loop_rate(1000); // 1ms
	while (ros::ok())
	{
		usLength = CollectUARTData((char*)chrBuffer);
		int chrIndex = 0;

		if (usLength > 0)
		{
			usRxLength += usLength;

			//Save data from comport to ringbuffer then update producer
			if (usLength < (pBufferEnd - pProducer))
			{
				memcpy((void*)pProducer, chrBuffer, usLength);
			}
			else //part of chrBuffer should be copied to begin of ringbuffer.
			{
				memcpy((void*)pProducer, chrBuffer, pBufferEnd - pProducer);
				chrIndex += (int)(pBufferEnd - pProducer);
				memcpy(pBufferStart, (char*)&chrBuffer + chrIndex, usLength - (pBufferEnd - pProducer));
			}
			//update producer 
			pProducer += usLength;

			//reach the end and rewind to begin
			if (pProducer > pBufferEnd)
				pProducer = pBufferStart + (pProducer - pBufferEnd);

			//Processing data
			if (usRxLength >= FRAME_MIN_SIZE)
			{

				if ((cid == CLASS_ID_HOSTCONTROL_CMD) && (mid == (CMD_ID_SABER_DATA_PACKET | 0x80)))
				{
					
					Process();
					imuMsg.linear_acceleration.x = saberDataHandle.accLinear.accX;
					imuMsg.linear_acceleration.y = saberDataHandle.accLinear.accY;
					imuMsg.linear_acceleration.z = saberDataHandle.accLinear.accZ;

					imuMsg.angular_velocity.x = saberDataHandle.gyroCal.gyroX;
					imuMsg.angular_velocity.y = saberDataHandle.gyroCal.gyroY;
					imuMsg.angular_velocity.z = saberDataHandle.gyroCal.gyroZ;

					imuMsg.orientation.w = saberDataHandle.quat.Q0.float_x;
					imuMsg.orientation.x = saberDataHandle.quat.Q1.float_x;
					imuMsg.orientation.y = saberDataHandle.quat.Q2.float_x;
					imuMsg.orientation.z = saberDataHandle.quat.Q3.float_x;

					imuMsg.header.stamp= ros::Time::now();
					imuMsg.header.frame_id = "imu_link";
					imuMsg.header.seq = seq;
					seq = seq + 1;

					pub.publish(imuMsg);					
				}				
			}			
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	getchar();
	return 0;
}

int print_flag = 0;

void Process()
{
	u8 * pCurrentHead = NULL;
	u8 * pCurrent = NULL;
	u8 CompletePackage = 0;
	u8 * pEndRing = NULL;
	u8 * pStartRing = NULL;
	int ret = 0;
	pEndRing = (u8*)(&ringBuf) + RING_BUFFER_SIZE;
	pStartRing = (u8*)&ringBuf;
	u32 needProcessedLen = 0;

	//copy ringbuff to tempbuf
	if (pProducer >= pCustomer)
	{
		unProcessBytes = (u32)(pProducer - pCustomer);

		if (unProcessBytes <= sizeof(tempbuf))
			memcpy((void*)&tempbuf, pCustomer, unProcessBytes);

	}
	else
	{

		unProcessBytes = (u32)(RING_BUFFER_SIZE - (pCustomer - pProducer));
		if ((u32)(pEndRing - pCustomer) <= sizeof(tempbuf))
			memcpy((void*)&tempbuf, (void*)pCustomer, pEndRing - pCustomer);

		if ((u32)(pProducer - pStartRing) <= (sizeof(tempbuf) - (pEndRing - pCustomer)))
			memcpy((void*)((u8*)&tempbuf + (pEndRing - pCustomer)), (void*)pStartRing, pProducer - pStartRing);

	}

	
	if (unProcessBytes < FRAME_MIN_SIZE)
	{
		return;
	}

	pCurrentHead = (u8*)&tempbuf;

	while ((unProcessBytes >= FRAME_MIN_SIZE) && (ret != ERROR_NOT_ENOUGH_LENGTH))
	{
		previous_bytes = unProcessBytes;
		if (unProcessBytes >= ATOM_HEADER_LEN)
		{
			pCurrent = pCurrentHead;
			ret = AtomCmd_SearchingFrame((u8**)&pCurrentHead, RING_BUFFER_SIZE, (u32*)&unProcessBytes, &needProcessedLen);
		}

		if (ret == FRAME_COMPLETE)
		{
			CompletePackage = 1;
			printf("RX: ");
			int i = 0;
			for (i = 0; i < (needProcessedLen); i++)
				printf("%02X ", pCurrentHead[i]);
			printf("\n");
		}

		else if (ret == FRAME_ERROR)
			CompletePackage = 0;
		else
			CompletePackage = 0;

		//Modify consumer if unProcessBytes had been changed.
		pCustomer += (pCurrentHead - pCurrent);
		if (pCustomer > pEndRing)
			pCustomer = pStartRing + (pCustomer - pEndRing);


		if (CompletePackage)
		{
			CompletePackage = 0;

			//Save unprocessBytes before processer.
			previous_bytes = unProcessBytes;
			processedBytes = needProcessedLen;
			ret = AtomCmd_Processer(pCurrentHead, (u8**)&pCustomer, ringBuf, RING_BUFFER_SIZE, (u32*)&needProcessedLen);

			//processedBytes = previous_bytes - unProcessBytes;
			pCurrentHead += processedBytes;
			unProcessBytes -= processedBytes;

		}
		else
		{
			continue;
		}
	}

}



/*-------------------Packet parsing---------------------------------*/
void DataPacketParser(u8 *pBuffer, u16 dataLen)
{
	u16 PID = 0;
	u8 *pData = pBuffer;
	u8 index = 0;
	u8 pl = 0;

	//reset saberDataHandle
	memset(&saberDataHandle, 0, sizeof(saberDataHandle));
	//printf("\n");
	while (index < dataLen)
	{
		PID = ((*((u16*)(pData + index))) & 0x7fff);
		pl = *(pData + index + 2);

if (PID == (SESSION_NAME_TEMPERATURE))
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.temperature.data, pData + index, PL_TEMPERTURE);
			saberDataHandle.temperature.dataID = PID;
			saberDataHandle.temperature.dataLen = pl;
			printf(" *** temperature:\t%11.4f *** \n", saberDataHandle.temperature.data);

			index += PL_TEMPERTURE;

		}
		else if (PID == (SESSION_NAME_RAW_ACC))
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accRawData.accX, pData + index, PL_RAW_DATA);
			saberDataHandle.accRawData.dataID = PID;
			saberDataHandle.accRawData.dataLen = pl;

			index += PL_RAW_DATA;

		}
		else if (PID == SESSION_NAME_RAW_GYRO)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.gyroRawData.gyroX, pData + index, PL_RAW_DATA);
			saberDataHandle.gyroRawData.dataID = PID;
			saberDataHandle.gyroRawData.dataLen = pl;
			index += PL_RAW_DATA;
		}
		else if (PID == SESSION_NAME_RAW_MAG)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.magRawData.magX, pData + index, PL_RAW_DATA);
			saberDataHandle.magRawData.dataID = PID;
			saberDataHandle.magRawData.dataLen = pl;
			index += PL_RAW_DATA;
		}
		else if (PID == SESSION_NAME_CAL_ACC)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accCal.accX, pData + index, PL_CAL_DATA);
			saberDataHandle.accCal.dataID = PID;
			saberDataHandle.accCal.dataLen = pl;
			index += PL_CAL_DATA;

			printf(" *** accCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.accCal.accX, saberDataHandle.accCal.accY, saberDataHandle.accCal.accZ);
		}
		else if (PID == SESSION_NAME_CAL_GYRO)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.gyroCal.gyroX, pData + index, PL_CAL_DATA);

			saberDataHandle.gyroCal.dataID = PID;
			saberDataHandle.gyroCal.dataLen = pl;
			index += PL_CAL_DATA;

			printf(" *** gyroCal:    \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.gyroCal.gyroX, saberDataHandle.gyroCal.gyroY, saberDataHandle.gyroCal.gyroZ);
		}
		else if (PID == SESSION_NAME_CAL_MAG)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.magCal.magX, pData + index, PL_CAL_DATA);
			saberDataHandle.magCal.dataID = PID;
			saberDataHandle.magCal.dataLen = pl;
			index += PL_CAL_DATA;

			printf(" *** magCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.magCal.magX, saberDataHandle.magCal.magY, saberDataHandle.magCal.magZ);
		}
		else if (PID == SESSION_NAME_KAL_ACC)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accKal.accX, pData + index, PL_KAL_DATA);
			saberDataHandle.accKal.dataID = PID;
			saberDataHandle.accKal.dataLen = pl;
			index += PL_KAL_DATA;

		}
		else if (PID == SESSION_NAME_KAL_GYRO)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.gyroKal.gyroX, pData + index, PL_KAL_DATA);
			saberDataHandle.gyroKal.dataID = PID;
			saberDataHandle.gyroKal.dataLen = pl;
			index += PL_KAL_DATA;
		}
		else if (PID == SESSION_NAME_KAL_MAG)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.magKal.magX, pData + index, PL_KAL_DATA);
			saberDataHandle.magKal.dataID = PID;
			saberDataHandle.magKal.dataLen = pl;
			index += PL_KAL_DATA;
		}
		//////////////////////////
		else if (PID == SESSION_NAME_QUAT)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.quat.Q0.uint_x, pData + index, PL_QUAT_EULER);
			saberDataHandle.quat.dataID = PID;
			saberDataHandle.quat.dataLen = pl;
			index += PL_QUAT_EULER;
			printf(" *** quat :      \t%11.4f, %11.4f, %11.4f, %11.4f *** \n", saberDataHandle.quat.Q0.float_x, saberDataHandle.quat.Q1.float_x, saberDataHandle.quat.Q2.float_x, saberDataHandle.quat.Q3.float_x);

		}
		else if (PID == SESSION_NAME_EULER)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.euler.roll, pData + index, PL_QUAT_EULER);
			saberDataHandle.euler.dataID = PID;
			saberDataHandle.euler.dataLen = pl;
			index += PL_QUAT_EULER;
		
			//printf(" *** euler:      \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.euler.roll, saberDataHandle.euler.pitch, saberDataHandle.euler.yaw);
		}

		else if (PID == SESSION_NAME_ROTATION_M)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.romatix.a, pData + index, PL_MATERIX);
			saberDataHandle.romatix.dataID = PID;
			saberDataHandle.romatix.dataLen = pl;
			index += PL_MATERIX;

		}

		else if (PID == SESSION_NAME_LINEAR_ACC)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.accLinear.accX, pData + index, PL_LINEAR_ACC_DATA);
			saberDataHandle.accLinear.dataID = PID;
			saberDataHandle.accLinear.dataLen = pl;
			index += PL_LINEAR_ACC_DATA;
			
			printf(" *** lin_acc:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle.accLinear.accX, saberDataHandle.accLinear.accY, saberDataHandle.accLinear.accZ);
		}
		else if (PID == SESSION_NAME_PACKET_COUNTER)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.packetCounter.packerCounter, pData + index, PL_PACKET_NUMBER);
			saberDataHandle.packetCounter.dataID = PID;
			saberDataHandle.packetCounter.dataLen = pl;
			index += PL_PACKET_NUMBER;
		}
		else if (PID == SESSION_NAME_DELTA_T)
		{
			//Ignore pid and pl
			index += 3;
			memcpy(&saberDataHandle.dt.DT, pData + index, PL_DT_DATA);

			saberDataHandle.dt.dataID = PID;
			saberDataHandle.dt.dataLen = pl;
			index += PL_DT_DATA;
		}

		else if (PID == SESSION_NAME_OS_TIME)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.tick.OS_Time_ms, pData+index, PL_OS_REFERENCE_TIME-2); //first 4 bytes are miliseconds
			saberDataHandle.tick.OS_Time_ms = *((u32*)(pData + index));
			saberDataHandle.tick.OS_Time_us = *((u16*)(pData + index + 4));

			saberDataHandle.tick.dataID = PID;
			saberDataHandle.tick.dataLen = pl;
			index += PL_OS_REFERENCE_TIME;
		}
		else if (PID == SESSION_NAME_STATUS_WORD)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.status.status, pData + index, PL_STATUS);
			saberDataHandle.status.dataID = PID;
			saberDataHandle.status.dataLen = pl;
			index += PL_STATUS;
		}
		else if (PID == SESSION_NAME_PACKET_COUNTER)
		{
			//Ignore pid and pl
			index += 3;

			memcpy(&saberDataHandle.packetCounter.packerCounter, pData + index, PL_PACKET_NUMBER);
			saberDataHandle.packetCounter.dataID = PID;
			saberDataHandle.packetCounter.dataLen = pl;
			index += PL_PACKET_NUMBER;

			printf(" *** packet_count:      \t%d, *** \n", saberDataHandle.packetCounter.packerCounter);

		}
	}
}
/*-----------------------Packet selection---------------------------*/
void SelectPackets(char enable)
{
	u16 pid = 0;
	ConfigSingleDataPacket_HandleType Packet[6];
	u8 index = 0;
	int i = 0;
	for (i = 0; i < 5; i++)
	{
		switch (i)
			{
			case CAL_GYRO:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_CAL_GYRO | 0x8000;
				else
					pid = SESSION_NAME_CAL_GYRO;
				break;
			case LINEAR_ACC:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_LINEAR_ACC | 0x8000;
				else
					pid = SESSION_NAME_LINEAR_ACC;
				break;
			case EULER_DATA:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_EULER | 0x8000;
				else
					pid = SESSION_NAME_EULER;
				break;
			case QUAT_DATA:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_QUAT | 0x8000;
				else
					pid = SESSION_NAME_QUAT;
				break;
			case PACKET_COUNT:
				if (((enable >> i) & 0x1) == 0x1)
					pid = SESSION_NAME_PACKET_COUNTER | 0x8000;
				else
					pid = SESSION_NAME_PACKET_COUNTER;
				break;
			default:
				break;

			}
			Packet[index].reserve0 = 0xff;
			Packet[index].reserve1 = 0xff;
			Packet[index].packetID = pid;
			index++;
	}
	Atom_setDataPacketConfigReq((u8*)&Packet, index * 4);

}
