# 目录

- [目录](#目录)
- [STM32-Intelligent-Car](#stm32-intelligent-car)
- [源码框架：](#源码框架)
	- [MiniBalance.c](#minibalancec)
# STM32-Intelligent-Car
欢迎来到“stm32智能小车”仓库！这里是一个专注于基于STM32微控制器的智能小车项目的代码存储库。
我们致力于开发一个功能丰富且高度可定制的智能小车，利用STM32系列微控制器的强大功能，实现自主导航、传感器数据处理、远程控制等多种功能。  
主要特点： 
- 基于STM32微控制器，充分发挥其计算和控制能力。
- 配备各种传感器，包括但不限于超声波传感器、红外传感器、编码器等，用于环境感知和位置控制。
- 使用先进的算法实现自主导航和避障，使小车能够在复杂环境中智能移动。
- 提供多种远程控制方式，如手机App、蓝牙遥控等，方便用户操控小车。
- 开放源代码，您可以根据自己的需求进行定制和扩展，也欢迎贡献您的代码和想法。  
# 源码框架：

`USER`文件夹放置的是：入口文件

`CORE`文件夹放置的是：32的启动文件

`FWLIB`文件夹放置的是：ST官方的库文件

`SYSTEM`文件夹放置的是：延时函数，串口函数

`HAREWARE`文件夹放置的是：按键，led硬件相关

`MiniBalance`文件夹放置的是：控制函数和数据处理

## MiniBalance.c
```c
#include "stm32f10x.h"
#include "sys.h"
u8 Way_Angle=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //电机停止标志位和显示标志位  默认停止 显示打开
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //温度变量
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u32 Distance;                               //超声波测距
u8 delay_50,delay_flag,PID_Send; 						//延时和调参相关变量
u8 Flag_follow=0,Flag_avoid=0;							//超声波跟随、超声波壁障标志位
float Acceleration_Z;                       //Z轴加速度计  
float Balance_Kp=22500,Balance_Kd=108,Velocity_Kp=16000,Velocity_Ki=80,Turn_Kp=4200,Turn_Kd=0;//PID参数（放大100倍）
int main(void)
{ 
	MY_NVIC_PriorityGroupConfig(2);	//设置中断分组
	delay_init();	    	            //延时函数初始化	
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //初始化与 LED 连接的硬件接口
	KEY_Init();                     //按键初始化
	MiniBalance_PWM_Init(7199,0);   //初始化PWM 10KHZ与电机硬件接口，用于驱动电机
	uart_init(115200);	            //串口1初始化
	uart3_init(9600);             	//串口3初始化，用于蓝牙模块
	Encoder_Init_TIM2();            //编码器接口
	Encoder_Init_TIM4();            //初始化编码器4
	Adc_Init();                     //adc初始化
	IIC_Init();                     //IIC初始化
	OLED_Init();                    //OLED初始化	    
	MPU6050_initialize();           //MPU6050初始化	
	DMP_Init();                     //初始化DMP 
	TIM3_Cap_Init(0XFFFF,72-1);	    //超声波初始化
	MiniBalance_EXTI_Init();        //MPU6050 5ms定时中断初始化，节省定时器资源，减少cpu负担
	while(1)
	{
		if(Flag_Show==0)          		//使用MiniBalance APP和OLED显示屏
		{
			 APP_Show();								//发送数据给APP
			 oled_show();          			//显示屏打开
		}
		else                      		//使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
		{
			 DataScope();          			//开启MiniBalance上位机
		}	
		delay_flag=1;	
		delay_50=0;
		while(delay_flag);	     			//示波器需要50ms	高精度延时，delay函数不满足要求，故使用MPU6050中断提供50ms延时
	}
}
```

