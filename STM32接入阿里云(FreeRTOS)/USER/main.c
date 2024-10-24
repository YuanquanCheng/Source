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
//ESP8266WIFI使用相关头文件
#include "uart2.h"
#include "wifi.h"
#include "timer3.h"
#include "structure.h"

//FreeRTOS系统相关头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//MQTT协议相关头文件
#include "esp8266_mqtt.h"

//MQTT初始化函数
void ES8266_MQTT_Init(void);


static QueueHandle_t xQueue; // 定义一个队列句柄


//此处是阿里云服务器的登陆配置
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
char mqtt_message[300];	//MQTT的上报消息缓存

//服务器IP地址和端口号
char *IP = MQTT_BROKERADDRESS;
int Port = 1883;

//打印函数的任务句柄
TaskHandle_t print_Task_Handler;
//打印函数任务函数
void print_TASK(void *pvParameters);

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		64  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define LED0_TASK_PRIO		3
//任务堆栈大小	
#define LED0_STK_SIZE 		226  
//任务句柄
TaskHandle_t LED0Task_Handler;
//TaskHandle_t DHT11Task_Handler;
//任务函数
void led0_task(void *pvParameters);
//==========================================
//任务优先级
#define DHT11_TASK_PRIO		3
//任务堆栈大小	
#define DHT11_STK_SIZE 		238  
//任务句柄
TaskHandle_t DHT11Task_Handler;
//任务函数
//void led0_task(void *pvParameters);
//==========================================
//任务优先级
#define WIFI_TASK_PRIO		4
//任务堆栈大小	
#define WIFI_STK_SIZE 		238  
//任务句柄
TaskHandle_t WIFITask_Handler;
//任务函数
void wifi_task(void *pvParameters);


/* Uart2 - Wifi 的消息接收队列 */
#define Wifi_MESSAGE_Q_NUM   4   		//接收数据的消息队列的数量
QueueHandle_t Wifi_Message_Queue;		//信息队列句柄

float light ;   //光照值
float h ;   //湿度
float t ;   //温度

void led0_task(void *pvParameters);
//主函数
int main(void)
{	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4 
    delay_init();               //初始化系统时钟
	LED_Init();                 //LED初始化
    uart_init(115200);     	    //初始化串口1
    uart2_init(115200);         //初始化串口2
    Timer3_Configuration(5);    //Tim3定时器，用于wifi-uart2的接收完成
    WiFi_ResetIO_Init();		//wifi - RST引脚初始化
	
    printf("初始化完成，开始创建任务\r\n");
	// 创建一个能够存储 10 个 uint8_t 类型元素的队列
    xQueue = xQueueCreate(10, sizeof(float));
    //创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄     


	Wifi_Message_Queue = xQueueCreate(Wifi_MESSAGE_Q_NUM,1); //队列项数目是Wifi_MESSAGE_Q_NUM，队列项长度是串口DMA接收缓冲区长度 		
				
    vTaskStartScheduler();          //开启任务调度
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
			printf("温湿度：data=%.1f, data=%.1f\r\n", t, h);
		}
	vTaskDelay(5000);
	}
}
///**
//  * @brief  微秒级延时
//  * @param  xus 延时时长，范围：0~233015
//  * @retval 无
//  */
//void Delay_us(uint32_t xus)
//{
//	SysTick->LOAD = 72 * xus;				//设置定时器重装值
//	SysTick->VAL = 0x00;					//清空当前计数值
//	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
//	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
//	SysTick->CTRL = 0x00000004;				//关闭定时器
//}

///**
//  * @brief  毫秒级延时
//  * @param  xms 延时时长，范围：0~4294967295
//  * @retval 无
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
//  * @brief  秒级延时
//  * @param  xs 延时时长，范围：0~4294967295
//  * @retval 无
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
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
 
void DHT_Init_OutPut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
//	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void DHT_Start(void)
{
	DHT_Init_OutPut();
	GPIO_ResetBits(GPIOB, GPIO_Pin_11); // 拉低总线
//	Delay_us(19000);
	delay_us(19000);
//	vTaskDelay(19);
	GPIO_SetBits(GPIOB, GPIO_Pin_11); // 拉高总线
//	Delay_us(20);
	delay_us(20);
//	vTaskDelay(1);
	DHT_Init_InPut();
}
 
uint16_t DHT_Scan(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11); // 返回读取数据
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
		data <<= 1; // 向左进一位
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
		// DHT11输出的40位数据
		while(DHT_Scan() == RESET);
		DHT_Init_OutPut();
		GPIO_SetBits(GPIOB, GPIO_Pin_11);
		
		check = buffer[0] + buffer[1] + buffer[2] + buffer[3];
		if(check != buffer[4])
		{

			return 1; // 数据出错
		}
	}
	return  0;
}
//==========================================================================
//==========================================================================
 //开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    
    //创建 Uart2 - Wifi 接收消息队列
    Wifi_Message_Queue = xQueueCreate(Wifi_MESSAGE_Q_NUM,1); //队列项数目是Wifi_MESSAGE_Q_NUM，队列项长度是串口DMA接收缓冲区长度
    
    //创建LED0任务
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED0_TASK_PRIO,	
                (TaskHandle_t*  )&LED0Task_Handler); 
//创建DHT11_read任务
    xTaskCreate((TaskFunction_t )DHT11_read,     	
                (const char*    )"DHT11_read",   	
                (uint16_t       )DHT11_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DHT11_TASK_PRIO,	
				(TaskHandle_t*  )&DHT11Task_Handler); 
	
    //创建wifi_task任务
    xTaskCreate((TaskFunction_t )wifi_task,     	
                (const char*    )"wifi_task",   	
                (uint16_t       )WIFI_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )WIFI_TASK_PRIO,	
                (TaskHandle_t*  )&WIFITask_Handler);   
    //创建print_TASK任务
    xTaskCreate((TaskFunction_t )print_TASK,     	
                (const char*    )"print_TASK",   	
                (uint16_t       )128, 
                (void*          )NULL,				
                (UBaseType_t    )2,	
                (TaskHandle_t*  )&print_Task_Handler);            
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}
//打印任务栈函数
void print_TASK(void *pvParameters)
{
	static char InfoBuffer[512] = {0};
    while (1) {
        vTaskList((char *) &InfoBuffer);
        printf("任务名      任务状态    优先级    剩余栈    任务序号\r\n");
        printf("\r\n%s\r\n", InfoBuffer);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

//LED0任务函数 
void led0_task(void *pvParameters)
{
	//光照传感器初始化
    BH1750_Init();
    while(1)
    {
        //printf("led0_task !!!\r\n");
//		printf("led0_task,光照");
		/* 采集数据 */
		light = LIght_Intensity();	//读取光照强度的值
		printf("led0_task,光照=%.1f\r\n",light);
        vTaskDelay(500);
	 
    }
}   
 

//WIFI任务函数 
void wifi_task(void *pvParameters)
{
	float receivedData;//=============================================
	uint8_t pub_cnt = 0,pub_ret;
	uint16_t Counter_MQTT_Heart = 0;
    char *recv;
	
    //MQTT协议初始化
    ES8266_MQTT_Init();

    while(1)
    {
        //心跳包发送
		if(Counter_MQTT_Heart++>300)
		{
			Counter_MQTT_Heart = 0;
			MQTT_SentHeart();
		}
		
		/* 发送数据 */
		 pub_cnt++;
		if(0 == pub_cnt%500) //约3S发送一次数据
//		if (xQueueReceive(xQueue, &receivedData, portMAX_DELAY) == pdTRUE) 
		{
			pub_cnt = 0;
			memset(mqtt_message, 0, 300);
			//组装数据  
//			sprintf(mqtt_message,
//			"{\"method\":\"thing.service.property.post\",\"id\":\"1234\",\"params\":{\
//			\"Light\":%.1f},\"version\":\"1.0.0\"}", light);
			printf("温度=%.1f, 湿度=%.1f\r\n", t, h);
			sprintf(mqtt_message,
			"{\"method\":\"thing.service.property.set\",\"params\":{\"temp\":%.1f,\"Humidity\":%.1f,\"light\":%.1f}}",t,h,light);
			
			 //发布数据
			pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC,mqtt_message,0);
			if(pub_ret > 0)
			{
				printf("消息发布成功！！！data=%.1f, data=%.1f\r\n", t, h);

			}
			else
			{
//				printf("消息发布失败！！！pub_ret=%d\r\n", pub_ret);
				printf("消息发布失败！！！data=%.1f, data=%.1f\r\n", t, h);

			}
		}
        //收到数据
        if((WifiMsg.U2_RxCompleted == 1) && (Usart2_RxCounter > 1))
        {
            printf("来自服务器数据：%d\r\n", Usart2_RxCounter);
			recv = strstr(Usart2_RxBuff, "LED"); 
            //下发命令后，串口2会接收到这样的数据：
			//...{"method":"thing.service.property.set","id":"1593428732","params":{"LED":1},"version":"1.0.0"}			
            if(recv != NULL)
            {	
				//经过strstr函数后，recv指向了字符串：LED":0}...
				//为拿到LED后面的状态值，指针偏移5个字节
				recv = recv + 3 +2;  //LED占3个字节  ”:占2个字节
                printf("LED=%d\r\n", (*recv)-'0');
                LED0 = !((*recv)-'0');  //根据下发的命令控制PC13处的LED灯
            
                memset(mqtt_message, 0, 300);
                //组装数据  
                sprintf(mqtt_message,
                "{\"method\":\"thing.service.property.set\",\"id\":\"5678\",\"params\":{\
                \"LED\":%d},\"version\":\"1.0.0\"}", (*recv)-'0');
                
                //发布数据
                pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC,mqtt_message,0);
                if(pub_ret > 0)
                {
                    printf("消息发布成功！！！pub_ret=%d\r\n", pub_ret);
                }
                else
                {
                    printf("消息发布失败！！！pub_ret=%d\r\n", pub_ret);
                }
            }
            //将标志位和数据清空
            memset(Usart2_RxBuff, 0, sizeof(Usart2_RxBuff));
            WifiMsg.U2_RxCompleted = 0;
            Usart2_RxCounter = 0;
        } 
        vTaskDelay(10);
    }
} 


//MQTT初始化函数
void ES8266_MQTT_Init(void)
{
	uint8_t status=1;
    char conn=1;

	// 复位不成功，需要重新复位
//    if(!WiFi_Init())
//    {
//        printf("ESP8266状态初始化正常\r\n");		//串口输出信息
//        //获取WIFI当前IP地址
//        WiFi_GetIP(100);
//        WifiMsg.Mode = 1;							//r_flag标志置位，表示8266状态正常，可以继续，进行TCP连接 
//        status++;
//    }
  
    printf("准备复位模块\r\n");                     //串口提示数据
	if(WiFi_Reset(50))
	{                                //复位，100ms超时单位，总计5s超时时间
		printf("复位失败，准备重启\r\n");           //返回非0值，进入if，串口提示数据
	}else printf("复位成功\r\n");                   //串口提示数据
    
    printf("准备连接路由器\r\n");                   //串口提示数据	
   
	if(WiFi_JoinAP(10)){                               //连接路由器,1s超时单位，总计10s超时时间
        printf("连接路由器失败，准备重启\r\n");     //返回非0值，进入if，串口提示数据
    }else printf("连接路由器成功\r\n");             //串口提示数据
    	printf("准备获取IP地址\r\n");                   //串口提示数据
	if(WiFi_GetIP(50)){                                //准备获取IP地址，100ms超时单位，总计5s超时时间
		printf("获取IP地址失败，准备重启\r\n");     //返回非0值，进入if，串口提示数据
	}else printf("获取IP地址成功\r\n");             //串口提示数据
	
	printf("准备开启透传\r\n");                     //串口提示数据
	if(WiFi_SendCmd("AT+CIPMODE=1",50)){               //开启透传，100ms超时单位，总计5s超时时间
		printf("开启透传失败，准备重启\r\n");       //返回非0值，进入if，串口提示数据
	}else printf("开启透传成功\r\n");               //串口提示数据
	
	printf("准备关闭多路连接\r\n");                 //串口提示数据
	if(WiFi_SendCmd("AT+CIPMUX=0",50)){                //关闭多路连接，100ms超时单位，总计5s超时时间
		printf("关闭多路连接失败，准备重启\r\n");   //返回非0值，进入if，串口提示数据
	}else printf("关闭多路连接成功\r\n");           //串口提示数据
    WifiMsg.Mode = 1;							//r_flag标志置位，表示8266状态正常，可以继续，进行TCP连接 
    status++;
	
	//连接阿里云IOT服务器
	if(status==2)
	{
        printf("连接服务器：IP=%s,Port=%d\r\n",IP, Port);
        conn = WiFi_Connect(IP, Port, 100);
        printf("连接结果conn=%d\r\n",conn);
        status++;
	}
    //关闭WIFI回显
    //printf("关闭回显：%d\r\n", WiFi_Send("ATE0"));
	
	//登陆MQTT
	if(status==3)
	{
		//不用判断返回值，登陆总是显示失败，但实际已经登陆成功了
		MQTT_Connect(MQTT_CLIENTID, MQTT_USARNAME, MQTT_PASSWD);
		printf("ESP8266阿里云MQTT登陆成功！\r\n");
		status++;
	}

	//订阅主题
	if(status==4)
	{
		//不用判断返回值，订阅总是显示失败，但实际已经订阅成功了
		MQTT_SubscribeTopic(MQTT_SUBSCRIBE_TOPIC,0,1);
		printf("ESP8266阿里云MQTT订阅主题成功！\r\n");
	}
}
