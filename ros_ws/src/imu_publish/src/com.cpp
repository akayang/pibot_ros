//#include 	"Linux_Com.h"


#include     <stdio.h> 
#include 	<string.h>
#include     <stdlib.h> 
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h> 
#include     <termios.h> 
#include     <errno.h>
#include 	<pthread.h>
#include 	<unistd.h> 
#include    	<time.h>
#include <imu_publish/atom_macro.h>
#include <imu_publish/atomprotocol.h>

#define		iBufferSize 501
#define     UARTBufferLength 98304
#define 	u8	unsigned char

#define B9600	0000015	//9600 b/s
#define B38400	0000017	//38400 b/s
#define B57600 0010001	//57600 b/s
#define B115200 0010002	//115200 b/s
#define B230400 0010003	//230400 b/s
#define B460800 0010004	//460800 b/s
#define B500000 0010005	//500000 b/s
#define B576000 0010006	//576000 b/s
#define B921600 0010007	//921600 b/s
#define B1500000 0010012	//1500000 b/s
#define rxBufferSize 12
int dataReady;

unsigned char strRxBuf[rxBufferSize]; //接收要用unsigned
int nullRecvCounter=0;
int nullFirstRecvCounter=0;
pthread_t receiveDataThread;
static char chrUARTBuffers[UARTBufferLength]={0};
static unsigned long ulUARTBufferStart=0, ulUARTBufferEnd=0;

unsigned long uLen;
unsigned long ulLen1;
unsigned long ulLen2;
char chrBuffer_com[iBufferSize]={0};	

void Process();
int nFd;
void *ReceiveCOMData(void* arg);
signed char OpenCOMDevice(const  char *Port,const int iBaudrate);

unsigned short CollectUARTData(char chrUARTBufferOutput[])
{
	unsigned long ulLength=0;
	unsigned long ulEnd ;
	unsigned long ulStart ;

	ulEnd = ulUARTBufferEnd;
	ulStart = ulUARTBufferStart;
	if (ulEnd == ulStart)
		return(0);
	if (ulEnd > ulStart)
	{
		memcpy((void*)chrUARTBufferOutput,(void*)(chrUARTBuffers+ulStart),ulEnd-ulStart);
		ulLength = ulEnd-ulStart;
	}
	else
	{
		memcpy((void*)chrUARTBufferOutput,(void*)(chrUARTBuffers+ulStart),UARTBufferLength-ulStart);
		if ( ulEnd != 0 )
		{
			memcpy((void*)(chrUARTBufferOutput+(UARTBufferLength-ulStart)),(void*)chrUARTBuffers,ulEnd);
		}
		ulLength = UARTBufferLength+ulEnd-ulStart;
	}
	ulUARTBufferStart = ulEnd;
	return (unsigned short) ulLength;
}

unsigned char ATR_Linux_OpenCom(const char *Port,const int iBaudrate)
{
	unsigned char r;
	signed char ret=OpenCOMDevice(Port,iBaudrate);
	if(ret >= 0)
	{
		r = 0x00;
	}
	else
		r = 0x1C;
	return r;
}

void CloseCOMDevice(void)
{
	close(nFd);
}

void ATR_Linux_CloseCom()
{
	CloseCOMDevice();
}
				

signed char OpenCOMDevice(const  char *Port,const int iBaudrate)
{
	printf("Connecting %s\n",Port);
	nFd=open(Port, O_RDWR|O_NOCTTY|O_NDELAY );
	printf("nFd %d\n",nFd);
	if(nFd<0)
	{
		printf("Connect %s failed!\n",Port);
		return -1;
	}

	struct termios options = { 0 };   //宣告一個設定 comport 所需要的結構體 並且清空內部
	
	/* c_cflag 控制模式：
	 * CLOCAL:忽略任何modem status lines
	 * CREAD:啟動字元接收
	 * CS8:傳送或接收時，使用八個位元
	 */
	int setSpeed;

	switch(iBaudrate)
	{
		case 9600:	setSpeed=B9600;
				  	break;
		case 38400:	setSpeed=B38400;
					break;
		case 57600: setSpeed=B57600;
					break;
		case 115200:setSpeed=B115200;
					break;
		case 230400:setSpeed=B230400;
					break;
		case 460800:setSpeed=B460800;
					break;
		case 921600:setSpeed=B921600;
					break;
		case 1500000:setSpeed=B1500000;
					 break;
		default: setSpeed=B115200;
				break;
	}

	options.c_cflag = (setSpeed | CLOCAL | CREAD | CS8); //依序,設定 baud rate,不改變 comport 擁有者, 接收致能, 8 data bits

	cfsetispeed( &options , setSpeed );
	cfsetospeed( &options , setSpeed );

	options.c_cc[ VTIME ] = 1;	//10 = 1秒,定義等待的時間，單位是百毫秒
	options.c_cc[ VMIN ] = 0;	//定義了要求等待的最小字節數,這個基本上給 0
	tcflush(nFd , TCIOFLUSH );	// 刷新和立刻寫進去fd

	fcntl(nFd,F_SETFL,0);//zuse

	if ( (tcsetattr(nFd , TCSAFLUSH , &options )) == -1 )
	{ //寫回本機設備,TCSANOW >> 立刻改變數值
		nFd = -1;
	}

	//printf("%s connected!\n",pDev[PortNo]);
	printf("%s connected!\n",Port);

	if(pthread_create(&receiveDataThread, NULL, &ReceiveCOMData, NULL)!=0)
		printf("Create receive com data thread failed!\n");
	else
		printf("Create receive com data thread successfully!\n");

	return nFd;
}

void *ReceiveCOMData(void* arg)//Thread run function
{
	
	unsigned long ulUARTBufferEndTemp=ulUARTBufferEnd;

	

	while(1)
	{
		memset(chrBuffer_com,0,iBufferSize);
		uLen=read(nFd,chrBuffer_com,iBufferSize-1);

		if(uLen<=0)
		{
			continue;
			               
		}
		if(uLen>0)
		{
			dataReady=1;
		}

		if((ulUARTBufferEndTemp+uLen) > UARTBufferLength)
		{
			ulLen1=UARTBufferLength-ulUARTBufferEndTemp;
			ulLen2=uLen-ulLen1;

			if(ulLen1>0)
			{
				memcpy((void*)&chrUARTBuffers[ulUARTBufferEnd],(void*)chrBuffer_com,ulLen1);
			}
				

			if(ulLen2>0)
			{
				memcpy((void*)&chrUARTBuffers[0],(void*)(chrBuffer_com+ulLen1),ulLen2);
			}
				

			ulUARTBufferEndTemp=ulLen2;
		}
		else
		{
			
			memcpy((void*)&chrUARTBuffers[ulUARTBufferEnd], (void *)chrBuffer_com, uLen);
			ulUARTBufferEndTemp += uLen;
		}
		
		if(ulUARTBufferEndTemp == ulUARTBufferStart )
		{
			printf("Error!");
		}
		else
		{
			ulUARTBufferEnd = ulUARTBufferEndTemp;
		}


	}
}

signed char SendUARTMessageLength(const char chrSendBuffer[],const unsigned short usLen)
{
	write(nFd,chrSendBuffer,usLen);
	return 0;
}




signed char SetBaudrate(const int iBaudrate)
{
	struct termios options = { 0 };   //宣告一個設定 comport 所需要的結構體 並且清空內部
	options.c_cflag = (iBaudrate | CLOCAL | CREAD | CS8); //依序,設定 baud rate,不改變 comport 擁有者, 接收致能, 8 data bits

	cfsetispeed( &options , iBaudrate );
	cfsetospeed( &options , iBaudrate );

	if ( (tcsetattr( nFd , TCSANOW , &options )) == -1 )
		return -1;

	return 0;
}
