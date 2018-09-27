#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "lcd.h"
#include "hc05.h"
#include "usart2.h"			 	 
#include "string.h"	 
#include "key.h"
//ALIENTEKminiSTM32开发板扩展实验 
//ATK-HC05蓝牙串口模块实验-库函数版本  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司 
//ALIENTEK战舰STM32开发板实验13
//TFTLCD显示实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
void HC05_Progress(void);
void GetTemAndHum(void); 	
extern u8 raw_data[20],Receive_ok;	
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
	                                                                         	 //用于TIM3的CH2输出的PWM通过该LED显示
					
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_WriteBit(GPIOA, GPIO_Pin_7,Bit_SET); // PA7上拉	

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	//TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	//TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
	
  //TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
 
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
   
}
	//显示ATK-HC05模块的主从状态
void HC05_Role_Show(void)
{
//	if(HC05_Get_Role()==1)LCD_ShowString(30,140,200,16,16,"ROLE:Master");	//主机
//	else LCD_ShowString(30,140,200,16,16,"ROLE:Slave ");			 		//从机
}
//显示ATK-HC05模块的连接状态
void HC05_Sta_Show(void)
{												 
	if(HC05_LED)LCD_ShowString(120,10,120,16,16,"STA:Connected ");			//连接成功
	else LCD_ShowString(120,10,120,16,16,"STA:Disconnect");	 			//未连接				 
}
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_X;
  
  /* 4个抢占优先级，4个响应优先级 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  /*抢占优先级可打断中断级别低的中断*/
	/*响应优先级按等级执行*/
	NVIC_X.NVIC_IRQChannel = USART1_IRQn;//中断向量
  NVIC_X.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_X.NVIC_IRQChannelSubPriority = 0;//响应优先级
  NVIC_X.NVIC_IRQChannelCmd = ENABLE;//使能中断响应
  NVIC_Init(&NVIC_X);
}
//发送一个字节数据
//input:byte,待发送的数据
void USART1_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);//等待发送完成
	USART1->DR=byte;	
}
//发送多字节数据
void USART_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART1_send_byte(Buffer[i++]);
	}
}
//发送多字节数据+校验和
void USART_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//累加Length-1前的数据
		USART1_send_byte(Buffer[i++]);
	}
}
void send_com(u8 data)
{
	u8 bytes[3]={0};
	bytes[0]=0xa5;
	bytes[1]=data;//功能字节
	USART_Send(bytes,3);//发送帧头、功能字节、校验和
}
typedef struct
{
    uint32_t P;
    uint16_t Temp;
    uint16_t Hum;
    uint16_t Alt;
} bme;
u16 a,b,c,d;u8 flag[1]={0x41};
bme Bme={0,0,0,0};
int main(void)
 {
	
		delay_init();
		NVIC_Configuration();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
		LED_Init();				//初始化与LED连接的硬件接口
		KEY_Init();				//初始化按键
		uart_init(9600);
		delay_ms(100);//等待模块初始化完成
		send_com(0x82);//发送读气压温湿度指令
		LCD_Init();	
		LCD_ShowString(10,150,200,16,16,"Temp=             degree");
		LCD_ShowString(10,180,200,16,16,"Hum=");
		while(HC05_Init()) 		//初始化ATK-HC05模块  
		{
			LCD_ShowString(30,10,200,16,16,"ATK-HC05 Error!"); 
			delay_ms(500);
			LCD_ShowString(30,10,200,16,16,"Please Check!!!"); 
			delay_ms(100);
		}		
			LCD_ShowString(30,10,200,16,16,"WK_UP:ROLE KEY0:SEND/STOP");  
			LCD_ShowString(30,60,200,16,16,"Send:");	
			LCD_ShowString(30,80,200,16,16,"Receive:");
			POINT_COLOR=BLUE;
						
		while(1)
	{   
		HC05_Progress(); 		
		GetTemAndHum();
	}								    
}
void HC05_Progress(void){
	 u8 key;u8 sendmask=0;u8 sendcnt=0;u8 sendbuf[20];u8 reclen=0;u8 flag=5;u8 a=1;
	if(USART2_RX_STA&0X8000)			//接收到一次数据了
		{
			LCD_Fill(30,200,240,320,WHITE);	//清除显示
 			reclen=USART2_RX_STA&0X7FFF;	//得到数据长度
		  USART2_RX_BUF[reclen]=0;	 	//加入结束符
			
 			LCD_ShowString(80,70,209,119,16,USART2_RX_BUF);//显示接收到的数据
			if(USART2_RX_BUF[0]=='A'){
				LED1=!LED1;
				sprintf((char*)sendbuf,"%.2f",(float)Bme.Temp/100);
				u2_printf("%.2f",(float)Bme.Temp/100);
				delay_ms(500);
				USART_RX_BUF[0]=9;
			}
			else if(USART2_RX_BUF[0]=='B'){
				LED0=!LED0;
				sprintf((char*)sendbuf,"%.2f",(float)Bme.Temp/100);
				u2_printf("%.2f",(float)Bme.Hum/100);
				delay_ms(500);
				USART_RX_BUF[0]=9;
			}
			else if(USART2_RX_BUF[0]=='C'){
				LED1=1;
			}
			else if(USART2_RX_BUF[0]=='D'){
				LED1=0;
			}
			else if(USART2_RX_BUF[0]=='E'){
				a=1;flag=5;
				while(a){
					flag++;
					if(flag>=20){ 
						flag=20;
						a=0;
					}
					TIM1_PWM_Init(199,7199);//(7200*200)/72000000=0.02=20ms
					TIM_SetCompare2(TIM3,flag);
					delay_ms(30);				
				}
			}
			else if(USART2_RX_BUF[0]=='F'){
				a=1;flag=20;
				while(a){
					flag--;
					if(flag<=5){ 
						flag=5;
						a=0;
					}
					TIM1_PWM_Init(199,7199);//(7200*200)/72000000=0.02=20ms
					TIM_SetCompare2(TIM3,flag);
					delay_ms(30);	
				}
			}
		}
		USART2_RX_STA=0;	
}
void GetTemAndHum(void)
{
	u8 sum=0,i=0;int16_t data=0;
	uint16_t data_16[2]={0};
	float temperInt,temperXiao,HumInt,HumXiao,t;
	int aaa,bbb,ccc,ddd;
		if(Receive_ok)//串口接收完毕
		{
			for(sum=0,i=0;i<(raw_data[3]+4);i++)//rgb_data[3]=3
			sum+=raw_data[i];
			if(sum==raw_data[i])//校验和判断
			{
				Bme.Temp=(raw_data[4]<<8)|raw_data[5];
				data_16[0]=(((uint16_t)raw_data[6])<<8)|raw_data[7];
				data_16[1]=(((uint16_t)raw_data[8])<<8)|raw_data[9];
				Bme.P=(((uint32_t)data_16[0])<<16)|data_16[1];
        Bme.Hum=(raw_data[10]<<8)|raw_data[11];
        Bme.Alt=(raw_data[12]<<8)|raw_data[13]; 
				//send_3out(&raw_data[4],10,0x45);//上传给上位机
				//LCD_ShowString(30,30,200,16,16,"l am successful");
				a=(float)Bme.Temp/100;
				b=(float)Bme.P/100;
				c=(float)Bme.Hum/100;
				d=(float)Bme.Alt/100;
				aaa=(int)a;
				temperInt=(float)aaa;
				temperXiao=((float)Bme.Temp/100-temperInt)*100;
				aaa=(int)temperXiao;
				temperXiao=(float)aaa;		
				LCD_ShowxNum(100,150,temperInt,2,16,0);
				LCD_ShowxNum(125,150,temperXiao,2,16,0);
				LCD_ShowString(118,150,200,16,16,".");
				
				ccc=(int)c;
				HumInt=(float)ccc;
				HumXiao=((float)Bme.Hum/100-HumInt)*100;
				ccc=(int)HumXiao;
				HumXiao=(float)ccc;		
				LCD_ShowxNum(100,180,HumInt,2,16,0);
				LCD_ShowxNum(125,180,HumXiao,2,16,0);
				LCD_ShowString(118,180,200,16,16,".");
				
			  printf("Temp: %.2f  DegC  ",(float)Bme.Temp/100);
		    printf("  P: %.2f  Pa ",(float)Bme.P/100);
			  printf("  Hum: %.2f   ",(float)Bme.Hum/100);
		    printf("  Alt: %.2f  m\r\n ",(float)Bme.Alt);
			}
			Receive_ok=0;//处理数据完毕标志
		}
}
