#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "bh1750.h"
#include "string.h"
//=====DHT11===============
void DHT11_read(void);
void DHT_Init_InPut(void);
void DHT_Init_OutPut(void);
void DHT_Start(void);
uint16_t DHT_Scan(void);
uint16_t DHT_ReadBit(void);
uint16_t DHT_ReadByte(void);
uint16_t DHT_ReadData(uint8_t buffer[5]);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);
//=========================
//ESP8266WIFIʹ�����ͷ�ļ�
#include "uart2.h"
#include "wifi.h"
#include "timer3.h"
#include "structure.h"

//FreeRTOSϵͳ���ͷ�ļ�
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//MQTTЭ�����ͷ�ļ�
#include "esp8266_mqtt.h"

//MQTT��ʼ������
void ES8266_MQTT_Init(void);


static QueueHandle_t xQueue; // ����һ�����о��


//�˴��ǰ����Ʒ������ĵ�½����
//#define MQTT_BROKERADDRESS "a1VPyJEJRjJ.iot-as-mqtt.cn-shanghai.aliyuncs.com"
//#define MQTT_CLIENTID "a1VPyJEJRjJ.test01|securemode=2,signmethod=hmacsha256,timestamp=1648914857644|"
//#define MQTT_USARNAME "test01&a1VPyJEJRjJ"
//#define MQTT_PASSWD "76599f069aff42f644669310490cdc3a9830c5431da41a81ff1467722d2bdb1b"
//#define	MQTT_PUBLISH_TOPIC "/sys/a1VPyJEJRjJ/test01/thing/event/property/post"
//#define MQTT_SUBSCRIBE_TOPIC "/sys/a1VPyJEJRjJ/test01/thing/service/property/set"

//================================================================================
#define MQTT_BROKERADDRESS "iot-06z00ffbje8ir4b.mqtt.iothub.aliyuncs.com"
#define MQTT_CLIENTID "k0urhI6ywuP.ESP32|securemode=2,signmethod=hmacsha256,timestamp=1727070192199|"
#define MQTT_USARNAME "ESP32&k0urhI6ywuP"
#define MQTT_PASSWD "2485232b3003d965e1ec110535b137c078bc925b47580787d4b9151455a64143"
#define	MQTT_PUBLISH_TOPIC "/k0urhI6ywuP/ESP32/user/ESP32"
#define MQTT_SUBSCRIBE_TOPIC "/k0urhI6ywuP/ESP32/user/ESP32"
//==================================================================================
char mqtt_message[300];	//MQTT���ϱ���Ϣ����

//������IP��ַ�Ͷ˿ں�
char *IP = MQTT_BROKERADDRESS;
int Port = 1883;

//��ӡ������������
TaskHandle_t print_Task_Handler;
//��ӡ����������
void print_TASK(void *pvParameters);

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		64  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define LED0_TASK_PRIO		3
//�����ջ��С	
#define LED0_STK_SIZE 		226  
//������
TaskHandle_t LED0Task_Handler;
//TaskHandle_t DHT11Task_Handler;
//������
void led0_task(void *pvParameters);
//==========================================
//�������ȼ�
#define DHT11_TASK_PRIO		3
//�����ջ��С	
#define DHT11_STK_SIZE 		238  
//������
TaskHandle_t DHT11Task_Handler;
//������
//void led0_task(void *pvParameters);
//==========================================
//�������ȼ�
#define WIFI_TASK_PRIO		4
//�����ջ��С	
#define WIFI_STK_SIZE 		238  
//������
TaskHandle_t WIFITask_Handler;
//������
void wifi_task(void *pvParameters);


/* Uart2 - Wifi ����Ϣ���ն��� */
#define Wifi_MESSAGE_Q_NUM   4   		//�������ݵ���Ϣ���е�����
QueueHandle_t Wifi_Message_Queue;		//��Ϣ���о��

float light ;   //����ֵ
float h ;   //ʪ��
float t ;   //�¶�

void led0_task(void *pvParameters);
//������
int main(void)
{	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4 
    delay_init();               //��ʼ��ϵͳʱ��
	LED_Init();                 //LED��ʼ��
    uart_init(115200);     	    //��ʼ������1
    uart2_init(115200);         //��ʼ������2
    Timer3_Configuration(5);    //Tim3��ʱ��������wifi-uart2�Ľ������
    WiFi_ResetIO_Init();		//wifi - RST���ų�ʼ��
	
    printf("��ʼ����ɣ���ʼ��������\r\n");
	// ����һ���ܹ��洢 10 �� uint8_t ����Ԫ�صĶ���
    xQueue = xQueueCreate(10, sizeof(float));
    //������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������     


	Wifi_Message_Queue = xQueueCreate(Wifi_MESSAGE_Q_NUM,1); //��������Ŀ��Wifi_MESSAGE_Q_NUM����������Ǵ���DMA���ջ��������� 		
				
    vTaskStartScheduler();          //�����������
}
 
//===DHT11=======================================================================
void DHT11_read(void)
{
	uint8_t buffer[5];
	while(1)
	{
	if(DHT_ReadData(buffer) == 0)
		{
			h = buffer[0] + buffer[1] / 10.0;
			t = buffer[2] + buffer[3] / 10.0;
			printf("��ʪ�ȣ�data=%.1f, data=%.1f\r\n", t, h);
		}
	vTaskDelay(5000);
	}
}
///**
//  * @brief  ΢�뼶��ʱ
//  * @param  xus ��ʱʱ������Χ��0~233015
//  * @retval ��
//  */
//void Delay_us(uint32_t xus)
//{
//	SysTick->LOAD = 72 * xus;				//���ö�ʱ����װֵ
//	SysTick->VAL = 0x00;					//��յ�ǰ����ֵ
//	SysTick->CTRL = 0x00000005;				//����ʱ��ԴΪHCLK��������ʱ��
//	while(!(SysTick->CTRL & 0x00010000));	//�ȴ�������0
//	SysTick->CTRL = 0x00000004;				//�رն�ʱ��
//}

///**
//  * @brief  ���뼶��ʱ
//  * @param  xms ��ʱʱ������Χ��0~4294967295
//  * @retval ��
//  */
//void Delay_ms(uint32_t xms)
//{
//	while(xms--)
//	{
//		Delay_us(1000);
//	}
//}
// 
///**
//  * @brief  �뼶��ʱ
//  * @param  xs ��ʱʱ������Χ��0~4294967295
//  * @retval ��
//  */
//void Delay_s(uint32_t xs)
//{
//	while(xs--)
//	{
//		Delay_ms(1000);
//	}
//} 

void DHT_Init_InPut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // ��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
 
void DHT_Init_OutPut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
//	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // �������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void DHT_Start(void)
{
	DHT_Init_OutPut();
	GPIO_ResetBits(GPIOB, GPIO_Pin_11); // ��������
//	Delay_us(19000);
	delay_us(19000);
//	vTaskDelay(19);
	GPIO_SetBits(GPIOB, GPIO_Pin_11); // ��������
//	Delay_us(20);
	delay_us(20);
//	vTaskDelay(1);
	DHT_Init_InPut();
}
 
uint16_t DHT_Scan(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11); // ���ض�ȡ����
}
 
uint16_t DHT_ReadBit(void)
{
	while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == RESET);
//	Delay_us(40);
	delay_us(40);
//	vTaskDelay(1);
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == SET)
	{
		while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == SET);
		return 1;
	}
	else
	{
		return 0;
	}
}
 
uint16_t DHT_ReadByte(void)
{
	uint16_t i, data = 0;
	for(i = 0; i < 8; i++)
	{
		data <<= 1; // �����һλ
		data |= DHT_ReadBit();
	}
	return data;
}
 
uint16_t DHT_ReadData(uint8_t buffer[5])
{
	uint16_t i =0;
	uint8_t check;
	
	DHT_Start();
	if(DHT_Scan() == RESET)
	{
		while(DHT_Scan() == RESET);
		while(DHT_Scan() == SET);
		for(i = 0; i < 5; i++)
		{
			buffer[i] = DHT_ReadByte();
		}
		// DHT11�����40λ����
		while(DHT_Scan() == RESET);
		DHT_Init_OutPut();
		GPIO_SetBits(GPIOB, GPIO_Pin_11);
		
		check = buffer[0] + buffer[1] + buffer[2] + buffer[3];
		if(check != buffer[4])
		{

			return 1; // ���ݳ���
		}
	}
	return  0;
}
//==========================================================================
//==========================================================================
 //��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    
    //���� Uart2 - Wifi ������Ϣ����
    Wifi_Message_Queue = xQueueCreate(Wifi_MESSAGE_Q_NUM,1); //��������Ŀ��Wifi_MESSAGE_Q_NUM����������Ǵ���DMA���ջ���������
    
    //����LED0����
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED0_TASK_PRIO,	
                (TaskHandle_t*  )&LED0Task_Handler); 
//����DHT11_read����
    xTaskCreate((TaskFunction_t )DHT11_read,     	
                (const char*    )"DHT11_read",   	
                (uint16_t       )DHT11_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DHT11_TASK_PRIO,	
				(TaskHandle_t*  )&DHT11Task_Handler); 
	
    //����wifi_task����
    xTaskCreate((TaskFunction_t )wifi_task,     	
                (const char*    )"wifi_task",   	
                (uint16_t       )WIFI_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )WIFI_TASK_PRIO,	
                (TaskHandle_t*  )&WIFITask_Handler);   
    //����print_TASK����
    xTaskCreate((TaskFunction_t )print_TASK,     	
                (const char*    )"print_TASK",   	
                (uint16_t       )128, 
                (void*          )NULL,				
                (UBaseType_t    )2,	
                (TaskHandle_t*  )&print_Task_Handler);            
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
//��ӡ����ջ����
void print_TASK(void *pvParameters)
{
	static char InfoBuffer[512] = {0};
    while (1) {
        vTaskList((char *) &InfoBuffer);
        printf("������      ����״̬    ���ȼ�    ʣ��ջ    �������\r\n");
        printf("\r\n%s\r\n", InfoBuffer);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

//LED0������ 
void led0_task(void *pvParameters)
{
	//���մ�������ʼ��
    BH1750_Init();
    while(1)
    {
        //printf("led0_task !!!\r\n");
//		printf("led0_task,����");
		/* �ɼ����� */
		light = LIght_Intensity();	//��ȡ����ǿ�ȵ�ֵ
		printf("led0_task,����=%.1f\r\n",light);
        vTaskDelay(500);
	 
    }
}   
 

//WIFI������ 
void wifi_task(void *pvParameters)
{
	float receivedData;//=============================================
	uint8_t pub_cnt = 0,pub_ret;
	uint16_t Counter_MQTT_Heart = 0;
    char *recv;
	
    //MQTTЭ���ʼ��
    ES8266_MQTT_Init();

    while(1)
    {
        //����������
		if(Counter_MQTT_Heart++>300)
		{
			Counter_MQTT_Heart = 0;
			MQTT_SentHeart();
		}
		
		/* �������� */
		 pub_cnt++;
		if(0 == pub_cnt%500) //Լ3S����һ������
//		if (xQueueReceive(xQueue, &receivedData, portMAX_DELAY) == pdTRUE) 
		{
			pub_cnt = 0;
			memset(mqtt_message, 0, 300);
			//��װ����  
//			sprintf(mqtt_message,
//			"{\"method\":\"thing.service.property.post\",\"id\":\"1234\",\"params\":{\
//			\"Light\":%.1f},\"version\":\"1.0.0\"}", light);
			printf("�¶�=%.1f, ʪ��=%.1f\r\n", t, h);
			sprintf(mqtt_message,
			"{\"method\":\"thing.service.property.set\",\"params\":{\"temp\":%.1f,\"Humidity\":%.1f,\"light\":%.1f}}",t,h,light);
			
			 //��������
			pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC,mqtt_message,0);
			if(pub_ret > 0)
			{
				printf("��Ϣ�����ɹ�������data=%.1f, data=%.1f\r\n", t, h);

			}
			else
			{
//				printf("��Ϣ����ʧ�ܣ�����pub_ret=%d\r\n", pub_ret);
				printf("��Ϣ����ʧ�ܣ�����data=%.1f, data=%.1f\r\n", t, h);

			}
		}
        //�յ�����
        if((WifiMsg.U2_RxCompleted == 1) && (Usart2_RxCounter > 1))
        {
            printf("���Է��������ݣ�%d\r\n", Usart2_RxCounter);
			recv = strstr(Usart2_RxBuff, "LED"); 
            //�·�����󣬴���2����յ����������ݣ�
			//...{"method":"thing.service.property.set","id":"1593428732","params":{"LED":1},"version":"1.0.0"}			
            if(recv != NULL)
            {	
				//����strstr������recvָ�����ַ�����LED":0}...
				//Ϊ�õ�LED�����״ֵ̬��ָ��ƫ��5���ֽ�
				recv = recv + 3 +2;  //LEDռ3���ֽ�  ��:ռ2���ֽ�
                printf("LED=%d\r\n", (*recv)-'0');
                LED0 = !((*recv)-'0');  //�����·����������PC13����LED��
            
                memset(mqtt_message, 0, 300);
                //��װ����  
                sprintf(mqtt_message,
                "{\"method\":\"thing.service.property.set\",\"id\":\"5678\",\"params\":{\
                \"LED\":%d},\"version\":\"1.0.0\"}", (*recv)-'0');
                
                //��������
                pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC,mqtt_message,0);
                if(pub_ret > 0)
                {
                    printf("��Ϣ�����ɹ�������pub_ret=%d\r\n", pub_ret);
                }
                else
                {
                    printf("��Ϣ����ʧ�ܣ�����pub_ret=%d\r\n", pub_ret);
                }
            }
            //����־λ���������
            memset(Usart2_RxBuff, 0, sizeof(Usart2_RxBuff));
            WifiMsg.U2_RxCompleted = 0;
            Usart2_RxCounter = 0;
        } 
        vTaskDelay(10);
    }
} 


//MQTT��ʼ������
void ES8266_MQTT_Init(void)
{
	uint8_t status=1;
    char conn=1;

	// ��λ���ɹ�����Ҫ���¸�λ
//    if(!WiFi_Init())
//    {
//        printf("ESP8266״̬��ʼ������\r\n");		//���������Ϣ
//        //��ȡWIFI��ǰIP��ַ
//        WiFi_GetIP(100);
//        WifiMsg.Mode = 1;							//r_flag��־��λ����ʾ8266״̬���������Լ���������TCP���� 
//        status++;
//    }
  
    printf("׼����λģ��\r\n");                     //������ʾ����
	if(WiFi_Reset(50))
	{                                //��λ��100ms��ʱ��λ���ܼ�5s��ʱʱ��
		printf("��λʧ�ܣ�׼������\r\n");           //���ط�0ֵ������if��������ʾ����
	}else printf("��λ�ɹ�\r\n");                   //������ʾ����
    
    printf("׼������·����\r\n");                   //������ʾ����	
   
	if(WiFi_JoinAP(10)){                               //����·����,1s��ʱ��λ���ܼ�10s��ʱʱ��
        printf("����·����ʧ�ܣ�׼������\r\n");     //���ط�0ֵ������if��������ʾ����
    }else printf("����·�����ɹ�\r\n");             //������ʾ����
    	printf("׼����ȡIP��ַ\r\n");                   //������ʾ����
	if(WiFi_GetIP(50)){                                //׼����ȡIP��ַ��100ms��ʱ��λ���ܼ�5s��ʱʱ��
		printf("��ȡIP��ַʧ�ܣ�׼������\r\n");     //���ط�0ֵ������if��������ʾ����
	}else printf("��ȡIP��ַ�ɹ�\r\n");             //������ʾ����
	
	printf("׼������͸��\r\n");                     //������ʾ����
	if(WiFi_SendCmd("AT+CIPMODE=1",50)){               //����͸����100ms��ʱ��λ���ܼ�5s��ʱʱ��
		printf("����͸��ʧ�ܣ�׼������\r\n");       //���ط�0ֵ������if��������ʾ����
	}else printf("����͸���ɹ�\r\n");               //������ʾ����
	
	printf("׼���رն�·����\r\n");                 //������ʾ����
	if(WiFi_SendCmd("AT+CIPMUX=0",50)){                //�رն�·���ӣ�100ms��ʱ��λ���ܼ�5s��ʱʱ��
		printf("�رն�·����ʧ�ܣ�׼������\r\n");   //���ط�0ֵ������if��������ʾ����
	}else printf("�رն�·���ӳɹ�\r\n");           //������ʾ����
    WifiMsg.Mode = 1;							//r_flag��־��λ����ʾ8266״̬���������Լ���������TCP���� 
    status++;
	
	//���Ӱ�����IOT������
	if(status==2)
	{
        printf("���ӷ�������IP=%s,Port=%d\r\n",IP, Port);
        conn = WiFi_Connect(IP, Port, 100);
        printf("���ӽ��conn=%d\r\n",conn);
        status++;
	}
    //�ر�WIFI����
    //printf("�رջ��ԣ�%d\r\n", WiFi_Send("ATE0"));
	
	//��½MQTT
	if(status==3)
	{
		//�����жϷ���ֵ����½������ʾʧ�ܣ���ʵ���Ѿ���½�ɹ���
		MQTT_Connect(MQTT_CLIENTID, MQTT_USARNAME, MQTT_PASSWD);
		printf("ESP8266������MQTT��½�ɹ���\r\n");
		status++;
	}

	//��������
	if(status==4)
	{
		//�����жϷ���ֵ������������ʾʧ�ܣ���ʵ���Ѿ����ĳɹ���
		MQTT_SubscribeTopic(MQTT_SUBSCRIBE_TOPIC,0,1);
		printf("ESP8266������MQTT��������ɹ���\r\n");
	}
}
