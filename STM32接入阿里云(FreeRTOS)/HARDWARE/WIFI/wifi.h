/*-------------------------------------------------*/
/*                                                 */
/*              ����Wifi���ܵ�ͷ�ļ�               */
/*                                                 */
/*-------------------------------------------------*/

#ifndef __WIFI_H
#define __WIFI_H

#include "uart2.h"	       //������Ҫ��ͷ�ļ�
#include "structure.h"

extern char Connect_flag;  //�ⲿ����������ͬ����������״̬  0����û�����ӷ�����  1�������Ϸ�������

#define RESET_IO(x)    GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)x)  //PA4����WiFi�ĸ�λ

#define WiFi_printf			u2_printf			//����2���� WiFi
#define WiFi_RxCounter		Usart2_RxCounter	//����2���� WiFi
#define WiFi_RX_BUF			Usart2_RxBuff		//����2���� WiFi
#define WiFi_RXBUFF_SIZE	USART2_RXBUFF_SIZE	//����2���� WiFi
#define Delay_Ms			delay_ms

//#define SSID	"XIAOCHUN01"				  //·����SSID����
//#define PASS	"3118003167"				   //·��������
#define SSID	"qq"				  //·����SSID����
#define PASS	"1234567890"				   //·��������


void WiFi_ResetIO_Init(void);
char WiFi_SendCmd(char *cmd, int timeout);
char WiFi_Reset(int timeout);
char WiFi_JoinAP(int timeout);
char WiFi_Connect_Server(int timeout);
char WiFi_Connect(char *serverip, int serverport, int timeout);
char WiFi_Smartconfig(int timeout);
char WiFi_WaitAP(int timeout);
char WiFi_GetIP(int timeout);
char WiFi_Get_LinkSta(void);
char WiFi_Get_Data(char *data, char *len, char *id);
char WiFi_SendData(char id, char *databuff, int data_len, int timeout);
char WiFi_Close(int timeout);
char WiFi_Init(void);
char WiFi_Send(char *sendbuf);
char WiFi_Recv(char *recvbuf, int len, int timeout);

#endif


