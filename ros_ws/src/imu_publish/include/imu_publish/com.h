#ifndef __LINUX_COM_H
#define __LINUX_COM_H



signed char	SendUARTMessageLength(const char chrMessage[],const unsigned short usLen);
unsigned short CollectUARTData(char chrUARTBufferOutput[]);
signed char OpenCOMDevice(const char *Port,const int iBaudrate);
void *ReceiveCOMData(void* arg);
void CloseCOMDevice(void);
signed char SetBaudrate(const int iBaudrate);

unsigned char ATR_Linux_OpenCom(const char *Port,const int iBaudrate);
void ATR_Linux_CloseCom();


#endif
