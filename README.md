# 缓慢更新中，敬请期待...
<div align="center">
  <img src="https://gifdb.com/images/high/blue-product-update-animated-graphic-mkw1bxww4h7tbxmv.gif" width="200" height="200" />
  <img src="https://media.giphy.com/media/dLmEzHozhc9WbTkwPa/giphy.gif" width="200" height="200" />
</div>

# 目录

- [缓慢更新中，敬请期待...](#缓慢更新中敬请期待)
- [目录](#目录)
- [STM32-Intelligent-Car](#stm32-intelligent-car)
- [源码框架：](#源码框架)
	- [MiniBalance.c](#minibalancec)
	- [电机驱动与编码器速度获取](#电机驱动与编码器速度获取)
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

## 电机驱动与编码器速度获取

电机驱动芯片采用`tb6612`芯片，可以驱动两组电机

`tb6612`芯片的模块引脚图

![1.png](/static/README-image/1.png)

一个`a`组，一个`b`组，每一组都可以驱动一个电机。

以`a`组为例子，`a`组有`PWMA`，`AIN2`，`AIN1`三个引脚。

`PWMA`接入单片机，`AIN1`，`AIN2`是控制电机的正反转。

这里有一个真值表，可以看看它的极性如何。

![2.png](/static/README-image/2.png)

`PWMA`是单片机的`PWM`输出，当输出不同的`PWM`占空比的时候，我们的芯片`AO1`，`AO2`也会输出不同的电压，这个时候就达到了`PWMA`波控制电机转速的效果。

我们的编码器是电机上面的一个小圆盘，它有四根线，分别是两根电源线跟一个`A`相，`B`相，当电机转动的时候，编码器会输出一系列的脉冲，`A`相跟`B`相都分别输出一个脉冲，`A`相跟`B`相输出的脉冲是正交关系。我们怎么来获取它的速度呢？就是通过单位实践读取脉冲的个数来获得它的速度。

![3.png](/static/README-image/3.png)

`A`相和`B`相的正交关系可以拿来检测电机的正转和反转，打个比方，当`A`相提前`B`相`90°`的时候，就是正转，当`A`相落后`B`相`90°`的时候，就是反转。

STM32采用编码器模式三可以识别出电机的正反转情况。正转的时候我们的定时器就向上计数，反转的时候我们的编码器就向下计数，这样就实现了正反转的识别。

代码实现：

在`MiniBalance.c`中进行电机初始化

> `MiniBalance.c`

```C
MiniBalance_PWM_Init(7199,0);   //初始化PWM 10KHZ与电机硬件接口，用于驱动电机
```

在`control.c`中编写电机驱动函数 

> `control.c`文件中 `Set_Pwm`函数

```C
/**************************************************************************
Function: Assign to PWM register
Input   : motor_left：Left wheel PWM；motor_right：Right wheel PWM
Output  : none
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	    BIN1=1,			BIN2=0; //前进 
	else           		BIN1=0,			BIN2=1; //后退
	PWMB=myabs(motor_left);	
  if(motor_right>0)		AIN2=1,			AIN1=0;	//前进
    else 	            AIN2=0,			AIN1=1; //后退
	PWMA=myabs(motor_right);
}
```

电机驱动函数`set_pwm` 原理分析：

输入参数：左电机的`pwm`值，右电机的`pwm`值

我们设定`pwm`值大于0，小车就会前进，`pwm`值小于0，小车就会后退。

先进来有一个判断pwm是否大于0，

`if(motor_left>0)`

`pwm`大于0，就前进，小于0，就回退。

然后把`pwm`值赋值给单片机的`pwm`寄存器。

`PWMB=myabs(motor_left)`

> `motor.h`

```YAML
#define PWMB   TIM1->CCR4  //PA11
#define BIN2   PBout(12)
#define BIN1   PBout(13)
#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define PWMA   TIM1->CCR1  //PA8
```

它可以输出一个`PWM`波

我们调用电机驱动函数`set_pwm`就可以使电机实现正反转跟转速的变化。

电机驱动函数`set_pwm`在5ms的定时中断函数里面使用。

> `control.c`文件中 `EXTI15_10_IRQHandler`函数第`57`行

```C
/**************************************************************************
Function: Control function
Input   : none
Output  : none
函数功能：所有的控制代码都在这里面
         5ms外部中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步	
入口参数：无
返回  值：无				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//电压测量相关变量
	static u8 Flag_Target;																//控制函数相关变量，提供10ms基准
	int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平衡环PWM变量，速度环PWM变量，转向环PWM变
	if(INT==0)		
	{   
		EXTI->PR=1<<12;                           					//清除中断标志位   
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
		Encoder_Left=-Read_Encoder(2);            					//读取左轮编码器的值，前进为正，后退为负
		Encoder_Right=-Read_Encoder(4);           					//读取右轮编码器的值，前进为正，后退为负
																												//左轮A相接TIM2_CH1,右轮A相接TIM4_CH2,故这里两个编码器的极性相同
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//编码器读数转速度（mm/s）
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;  		//给主函数提供50ms的精准延时，示波器需要50ms高精度延时
		}
		if(Flag_Target==1)                        					//10ms控制一次
		{
			Voltage_Temp=Get_battery_volt();		    					//读取电池电压		
			Voltage_Count++;                       						//平均值计数器
			Voltage_All+=Voltage_Temp;              					//多次采样累积
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值		
			return 0;	                                               
		}                                         					//10ms控制一次
		Read_Distane();                           					//获取超声波测量距离值
		if(Flag_follow==0&&Flag_avoid==0)	Led_Flash(100);   //LED闪烁;常规模式 1s改变一次指示灯的状态	
		if(Flag_follow==1||Flag_avoid==1)	Led_Flash(0);     //LED常亮;超声波跟随/避障模式	
		Key();                                    					//扫描按键状态 单击双击可以改变小车运行状态
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		Turn_Pwm=Turn(Gyro_Turn);														//转向环PID控制     
		
		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //计算左轮电机最终PWM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //计算右轮电机最终PWM
																												//PWM值正数使小车前进，负数使小车后退
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM限幅
		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被拿起
			Flag_Stop=1;	                           					//如果被拿起就关闭电机
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被放下
			Flag_Stop=0;	                           					//如果被放下就启动电机
		Choose(Encoder_Left,Encoder_Right);									//转动右轮选择小车模式
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//如果不存在异常
			Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器  
	 }       	
	 return 0;	  
} 
```

编码器使用了定时器`2`和定时器`4`

```C
int main(){
    Encoder_Init_TIM2();            //编码器接口
    Encoder_Init_TIM4();           //初始化编码器4
} 
```

> `encoder.c`中`Encoder_Init_TIM2`函数

```C
/**************************************************************************
Function: Initialize TIM2 to encoder interface mode
Input   : none
Output  : none
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,
  TIM_ICPolarity_Rising);//使用编码器模式3
}
```
