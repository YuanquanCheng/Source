#include "esp8266_mqtt.h"
#include "wifi.h"
#include "string.h"
#include "usart.h"

//FreeRTOS系统相关头文件
#include "FreeRTOS.h"
#include "task.h"

//连接成功服务器回应 20 02 00 00
//客户端主动断开连接 e0 00
const uint8_t parket_connetAck[] = {0x20,0x02,0x00,0x00};
const uint8_t parket_disconnet[] = {0xe0,0x00};
const uint8_t parket_heart[] = {0xc0,0x00};
const uint8_t parket_heart_reply[] = {0xc0,0x00};
const uint8_t parket_subAck[] = {0x90,0x03};

volatile uint16_t MQTT_TxLen;

//MQTT发送数据
void MQTT_SendBuf(uint8_t *buf,uint16_t len)
{
    int i=0;  
    while((USART2->SR&0X40)==0);
	for(i = 0;i < len;i ++)
	{			
		//USART2->DR = Usart2_TxBuff[i];
        USART2->DR = buf[i];
		while((USART2->SR&0X40)==0);	
	}	
}	

//发送心跳包
void MQTT_SentHeart(void)
{
	MQTT_SendBuf((uint8_t *)parket_heart,sizeof(parket_heart));
}

//MQTT无条件断开
void MQTT_Disconnect()
{
	MQTT_SendBuf((uint8_t *)parket_disconnet,sizeof(parket_disconnet));
}

//MQTT初始化
void MQTT_Init(uint8_t *prx,uint16_t rxlen,uint8_t *ptx,uint16_t txlen)
{
	memset(Usart2_TxBuff,0,sizeof(Usart2_TxBuff)); //清空发送缓冲
	memset(Usart2_RxBuff,0,sizeof(Usart2_RxBuff)); //清空接收缓冲
	
	//无条件先主动断开
	MQTT_Disconnect();
    //HAL_Delay(100);
    vTaskDelay(100);
	MQTT_Disconnect();
    //HAL_Delay(100);
    vTaskDelay(100);
}

//MQTT连接服务器的打包函数
uint8_t MQTT_Connect(char *ClientID,char *Username,char *Password)
{
	int ClientIDLen = strlen(ClientID);
	int UsernameLen = strlen(Username);
	int PasswordLen = strlen(Password);
	int DataLen;
//    int i=0;
    
    uint8_t cnt=1;
    uint8_t wait;
	MQTT_TxLen=0;
	//可变报头+Payload  每个字段包含两个字节的长度标识
    DataLen = 10 + (ClientIDLen+2) + (UsernameLen+2) + (PasswordLen+2);

    //固定报头
    //控制报文类型
    Usart2_TxBuff[MQTT_TxLen++] = 0x10;		//MQTT Message Type CONNECT
	//剩余长度(不包括固定头部)
	do
	{
		uint8_t encodedByte = DataLen % 128;
		DataLen = DataLen / 128;
		// if there are more data to encode, set the top bit of this byte
		if ( DataLen > 0 )
			encodedByte = encodedByte | 128;
		Usart2_TxBuff[MQTT_TxLen++] = encodedByte;
	}while ( DataLen > 0 );
    	
	//可变报头
	//协议名
	Usart2_TxBuff[MQTT_TxLen++] = 0;        		// Protocol Name Length MSB    
	Usart2_TxBuff[MQTT_TxLen++] = 4;        		// Protocol Name Length LSB    
	Usart2_TxBuff[MQTT_TxLen++] = 'M';        	// ASCII Code for M    
	Usart2_TxBuff[MQTT_TxLen++] = 'Q';        	// ASCII Code for Q    
	Usart2_TxBuff[MQTT_TxLen++] = 'T';        	// ASCII Code for T    
	Usart2_TxBuff[MQTT_TxLen++] = 'T';        	// ASCII Code for T    
	//协议级别
	Usart2_TxBuff[MQTT_TxLen++] = 4;        		// MQTT Protocol version = 4    
	//连接标志
	Usart2_TxBuff[MQTT_TxLen++] = 0xc2;        	// conn flags 
	Usart2_TxBuff[MQTT_TxLen++] = 0;        	// Keep-alive Time Length MSB    
	Usart2_TxBuff[MQTT_TxLen++] = 60;        	// Keep-alive Time Length LSB  60S心跳包  

	Usart2_TxBuff[MQTT_TxLen++] = BYTE1(ClientIDLen);// Client ID length MSB    
	Usart2_TxBuff[MQTT_TxLen++] = BYTE0(ClientIDLen);// Client ID length LSB  	
	memcpy(&Usart2_TxBuff[MQTT_TxLen],ClientID,ClientIDLen);
	MQTT_TxLen += ClientIDLen;
	
	if(UsernameLen > 0)
	{   
		Usart2_TxBuff[MQTT_TxLen++] = BYTE1(UsernameLen);		//username length MSB    
		Usart2_TxBuff[MQTT_TxLen++] = BYTE0(UsernameLen);    	//username length LSB    
		memcpy(&Usart2_TxBuff[MQTT_TxLen],Username,UsernameLen);
		MQTT_TxLen += UsernameLen;
	}
	
	if(PasswordLen > 0)
	{    
		Usart2_TxBuff[MQTT_TxLen++] = BYTE1(PasswordLen);		//password length MSB    
		Usart2_TxBuff[MQTT_TxLen++] = BYTE0(PasswordLen);    	//password length LSB  
		memcpy(&Usart2_TxBuff[MQTT_TxLen],Password,PasswordLen);
		MQTT_TxLen += PasswordLen; 
	}    
	
    //打印测试
//    printf("Usart2_RxBuff:\r\n");
//    for(i=0;i<MQTT_TxLen;i++)
//    {
//        printf("%0x", Usart2_TxBuff[i]);
//    }
//    printf("\r\n");

	while(cnt)
	{
        cnt--;
		memset(Usart2_RxBuff,0,sizeof(Usart2_RxBuff));
		MQTT_SendBuf((uint8_t *)Usart2_TxBuff,MQTT_TxLen);
		wait=30;//等待3s时间
		while(wait--)
		{
			//CONNECT
			if(Usart2_RxBuff[0]==parket_connetAck[0] && Usart2_RxBuff[1]==parket_connetAck[1]) //连接成功			   
			{
				return 1;//连接成功
			}
			//HAL_Delay(100);	
            vTaskDelay(100);
		}
	}
	return 0;
}

//MQTT订阅/取消订阅数据打包函数
//topic       主题 
//qos         消息等级 
//whether     订阅/取消订阅请求包
uint8_t MQTT_SubscribeTopic(char *topic,uint8_t qos,uint8_t whether)
{    
	int topiclen;
	int DataLen;
//	int i=0;
    
    uint8_t cnt=2;
	uint8_t wait;
    
	MQTT_TxLen=0;
    topiclen = strlen(topic);
    DataLen = 2 + (topiclen+2) + (whether?1:0);//可变报头的长度（2字节）加上有效载荷的长度
    
    //固定报头
	//控制报文类型
	if(whether) Usart2_TxBuff[MQTT_TxLen++] = 0x82; //消息类型和标志订阅
	else	Usart2_TxBuff[MQTT_TxLen++] = 0xA2;    //取消订阅

	//剩余长度
	do
	{
		uint8_t encodedByte = DataLen % 128;
		DataLen = DataLen / 128;
		// if there are more data to encode, set the top bit of this byte
		if ( DataLen > 0 )
			encodedByte = encodedByte | 128;
		Usart2_TxBuff[MQTT_TxLen++] = encodedByte;
	}while ( DataLen > 0 );	
	
	//可变报头
	Usart2_TxBuff[MQTT_TxLen++] = 0;				//消息标识符 MSB
	Usart2_TxBuff[MQTT_TxLen++] = 0x01;           //消息标识符 LSB
	//有效载荷
	Usart2_TxBuff[MQTT_TxLen++] = BYTE1(topiclen);//主题长度 MSB
	Usart2_TxBuff[MQTT_TxLen++] = BYTE0(topiclen);//主题长度 LSB   
	memcpy(&Usart2_TxBuff[MQTT_TxLen],topic,topiclen);
	MQTT_TxLen += topiclen;

	if(whether)
	{
		Usart2_TxBuff[MQTT_TxLen++] = qos;//QoS级别
	}
    
//    //打印测试
//    printf("Usart2_RxBuff:\r\n");
//    for(i=0;i<MQTT_TxLen;i++)
//    {
//        printf("%0x", Usart2_TxBuff[i]);
//    }
//    printf("\r\n");
	
	while(cnt)
	{
        cnt--;
		memset(Usart2_RxBuff,0,sizeof(Usart2_RxBuff));
		MQTT_SendBuf((uint8_t *)Usart2_TxBuff,MQTT_TxLen);
		wait=30;//等待3s时间
		while(wait--)
		{
			if(Usart2_RxBuff[0]==parket_subAck[0] && Usart2_RxBuff[1]==parket_subAck[1]) //订阅成功			   
			{
				return 1;//订阅成功
			}
			//HAL_Delay(100);	
            vTaskDelay(100);
		}
        //printf("rx[0]:%d,rx[1]:%d\r\n", Usart2_RxBuff[0], Usart2_RxBuff[1]);
	}
	if(cnt) return 1;	//订阅成功
	return 0;
}


//MQTT发布数据打包函数
//topic   主题 
//message 消息
//qos     消息等级 
uint8_t MQTT_PublishData(char *topic, char *message, uint8_t qos)
{  
	int topicLength = strlen(topic);    
	int messageLength = strlen(message);     
	static uint16_t id=0;
	int DataLen;
	MQTT_TxLen=0;
	//有效载荷的长度这样计算：用固定报头中的剩余长度字段的值减去可变报头的长度
	//QOS为0时没有标识符
	//数据长度             主题名   报文标识符   有效载荷
	if(qos)	DataLen = (2+topicLength) + 2 + messageLength;       
	else	DataLen = (2+topicLength) + messageLength;   

    //固定报头
	//控制报文类型
	Usart2_TxBuff[MQTT_TxLen++] = 0x30;    // MQTT Message Type PUBLISH  

	//剩余长度
	do
	{
		uint8_t encodedByte = DataLen % 128;
		DataLen = DataLen / 128;
		// if there are more data to encode, set the top bit of this byte
		if ( DataLen > 0 )
			encodedByte = encodedByte | 128;
		Usart2_TxBuff[MQTT_TxLen++] = encodedByte;
	}while ( DataLen > 0 );	
	
	Usart2_TxBuff[MQTT_TxLen++] = BYTE1(topicLength);//主题长度MSB
	Usart2_TxBuff[MQTT_TxLen++] = BYTE0(topicLength);//主题长度LSB 
	memcpy(&Usart2_TxBuff[MQTT_TxLen],topic,topicLength);//拷贝主题
	MQTT_TxLen += topicLength;
        
	//报文标识符
	if(qos)
	{
			Usart2_TxBuff[MQTT_TxLen++] = BYTE1(id);
			Usart2_TxBuff[MQTT_TxLen++] = BYTE0(id);
			id++;
	}
	memcpy(&Usart2_TxBuff[MQTT_TxLen],message,messageLength);
  MQTT_TxLen += messageLength;
        
	MQTT_SendBuf((uint8_t *)Usart2_TxBuff,MQTT_TxLen);
  return MQTT_TxLen;
}



