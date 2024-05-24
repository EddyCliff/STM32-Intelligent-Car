/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "usart3.h"
u8 Usart3_Receive;
/**************************************************************************
Function: Usart3 initialization
Input   : bound:Baud rate
Output  : none
函数功能：串口3初始化
入口参数：bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能UGPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3时钟
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 

}

/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{	  
	  static	int uart_receive=0;//蓝牙接收相关变量
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
  	uart_receive=USART_ReceiveData(USART3); 
		Usart3_Receive=uart_receive;
		if(uart_receive==0x59)  Flag_velocity=2;  //低速挡（默认值）
		if(uart_receive==0x58)  Flag_velocity=1;  //高速档
		
	  if(uart_receive>10)  //默认使用
    {			
			if(uart_receive==0x5A)	    Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else if(uart_receive==0x41)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
			else if(uart_receive==0x45)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
																	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //右
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    
																	Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //左
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
  	}
		if(uart_receive<10)     //备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
		{			
			Flag_velocity=1;//切换至高速档
			if(uart_receive==0x00)	Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
			else if(uart_receive==0x01)	Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;//前
			else if(uart_receive==0x05)	Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;//后
			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
														Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //左
			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //右
														Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;
			else Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;//刹车
  	}	

		
		if(Usart3_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
		if(Usart3_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位

		 if(Flag_PID==1)  //采集数据
		 {
				Receive[i]=Usart3_Receive;
				i++;
		 }
		 if(Flag_PID==2)  //分析数据
		 {
			  if(Receive[3]==0x50) 				 PID_Send=1;
			  else if(Receive[1]!=0x23) 
				{								
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
					switch(Receive[1])
					{
						case 0x30:  Balance_Kp=Data;break;
						case 0x31:  Balance_Kd=Data;break;
						case 0x32:  Velocity_Kp=Data;break;
						case 0x33:  Velocity_Ki=Data;break;
						case 0x34:  Turn_Kp=Data;break; 
					  case 0x35:  Turn_Kd=Data;break; 
						case 0x36:  break; //预留
						case 0x37:  break; //预留
						case 0x38:  break; //预留
					}
				}				 
			    Flag_PID=0;
					i=0;
					j=0;
					Data=0;
					memset(Receive, 0, sizeof(u8)*50);//数组清零
		 } 
	}  											 
} 

