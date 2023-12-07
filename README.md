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
		- [编码器的实现](#编码器的实现)
	- [角度，角速度获取与滤波算法](#角度角速度获取与滤波算法)
	- [PID控制](#pid控制)
	- [蓝牙遥控功能](#蓝牙遥控功能)
	- [超声波测距](#超声波测距)
	- [感谢](#感谢)
# STM32-Intelligent-Car
欢迎来到“stm32智能小车”仓库！这里是一个专注于基于STM32微控制器的智能小车项目的代码存储库。
 
主要特点： 
- 基于STM32微控制器。
- 配备各种传感器，包括但不限于超声波传感器、红外传感器、编码器等，用于环境感知和位置控制。
- 使用算法实现自主导航和避障，使小车能够在复杂环境中智能移动。
- 提供多种远程控制方式，如手机App、蓝牙遥控等，方便用户操控小车。
- 本小车为裸机开发，不带操作系统（RTOS，Linux等等） 

## 源码框架：

`USER`文件夹放置的是：入口文件

`CORE`文件夹放置的是：32的启动文件

`FWLIB`文件夹放置的是：ST官方的库文件

`SYSTEM`文件夹放置的是：延时函数，串口函数

`HAREWARE`文件夹放置的是：按键，led硬件相关

`MiniBalance`文件夹放置的是：控制函数和数据处理



##  MiniBalance.c

```C
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

> 电机驱动函数`set_pwm`在`control.c`中 `EXTI15_10_IRQHandler`函数被调用（第`20`行）

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
        //省略部分代码
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//如果不存在异常
			Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器 
	 }       	
	 return 0;	  
}
```



### 编码器的实现

编码器使用了定时器`2`和定时器`4`

> `MiniBalance.c`中`main`函数

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
  //省略部分代码
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,
  TIM_ICPolarity_Rising);//使用编码器模式3
}
```

使用编码器模式3

正转则向上计数，反转则向下计数。采用四倍频计数，就是编码器的上升沿和下降沿都计数。

AB相的脉冲有两个上升沿跟两个下降沿，是一个正交关系。

电机如果一圈输出390个脉冲，我们转动一圈就会有390x4，1560个计数。



初始化之后，我们来看读取编码器的函数。

> `encoder.c`中`Read_Encoder`函数

> 功能：读取编码器

```C
/**************************************************************************
Function: Read encoder count per unit time
Input   : TIMX：Timer
Output  : none
函数功能：单位时间读取编码器计数
入口参数：TIMX：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		 default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
```

这段代码是用于读取编码器计数值的函数。它具有以下功能和参数：

- **功能**：该函数的主要功能是读取编码器的计数值，并返回速度值。在函数内部，它会根据输入的定时器号`（TIMX）`选择相应的定时器`（TIM2、TIM3、TIM4）`来读取计数值，并将计数值清零。

- **入口参数**：函数接受一个参数 `TIMX`，表示要读取计数值的定时器号。根据这个参数，函数会选择相应的定时器来读取编码器计数。

- **返回值**：函数返回一个整数，表示读取的编码器计数值，即速度值。

函数首先根据输入的 `TIMX` 参数，选择要使用的定时器，然后读取该定时器的计数值。接着，它将该计数值清零，以便下次读取时能够获得正确的速度差值。

此函数的主要作用是测量编码器的速度，通常用于控制系统中的闭环反馈控制。当你调用这个函数并传递正确的 `TIMX` 参数时，它将返回相应定时器的编码器计数值，你可以根据这个值来计算速度。



> `Read_Encoder`函数在`control.c`中`EXTI15_10_IRQHandler`函数被调用（第`22,23`行）

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
		//省略部分代码
	 }       	
	 return 0;	  
} 
```



## 角度，角速度获取与滤波算法

姿态传感器采用`MPU6050`，它集成了一个加速度计和一个陀螺仪。

![4.png](/static/README-image/4.png)

角速度获取：

我们的角速度可以直接通过陀螺仪获取，就是直接读它的寄存器。

角度获取：

方法一：通过陀螺仪输出的角速度进行积分。

方法二：通过加速度计的分量来计算。

代码实现：

> 文件位置：`MiniBalance.c`中`main`函数 调用`MPU6050_initialize`函数

> 功能：`MPU6050`初始化

```C
int main(){
MPU6050_initialize();           //MPU6050初始化	
}
```

> 文件位置：`MPU6050.c`中`MPU6050_initialize`函数

```C
/**************************************************************************
Function: initialization Mpu6050 to enter the available state
Input   : none
Output  : none
函数功能：初始化	MPU6050 以进入可用状态
入口参数：无
返回  值：无
**************************************************************************/
void MPU6050_initialize(void) {
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //设置时钟
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪量程设置
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
    MPU6050_setSleepEnabled(0); //进入工作状态
	  MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	  MPU6050_setI2CBypassEnabled(0);	 //主控制器的I2C与	MPU6050的AUXI2C	直通关闭
}
```

这段代码是用于初始化 MPU6050 传感器的函数，使其进入可用状态。它具有以下功能和参数：

- **功能**：该函数的主要功能是初始化 MPU6050 传感器，使其处于可用状态。在函数内部，它调用了一系列 MPU6050 配置函数，包括设置时钟源、设置陀螺仪量程、设置加速度度量程、设置睡眠状态等。

- **入口参数**：函数没有接受任何入口参数。

- **返回值**：函数没有返回值，因为它主要是用来初始化传感器状态的。

函数首先调用 `MPU6050_setClockSource` 函数来设置 MPU6050 的时钟源，然后使用 `MPU6050_setFullScaleGyroRange` 和 `MPU6050_setFullScaleAccelRange` 分别设置陀螺仪和加速度计的量程。就是正负的`2000`度，因为陀螺仪和加速度计的数据是使用两个八位的寄存器储存的，就是十六位，然后它的读数应该是正负的`326768`。读到的原始数据是正`326768`的话就代表，它的角速度值就是正`2000`度，所以我们可以求出它的灵敏度，就是`32768÷2000=16.4`，就是`16.4`的灵敏度，就是每一度有`16.4`个读数。

接着，它调用 `MPU6050_setSleepEnabled` 函数将 MPU6050 设置为工作状态（非睡眠状态）。最后，它通过 `MPU6050_setI2CMasterModeEnabled` 和 `MPU6050_setI2CBypassEnabled` 函数来配置 MPU6050 的 I2C 控制方式，以及是否允许主控制器的 I2C 与 MPU6050 的 AUXI2C 直通。

这个函数的目的是将 MPU6050 配置为可用状态，以便后续读取传感器数据或进行其他操作。如果你希望使用 MPU6050 传感器，通常需要在程序初始化时调用这个函数。



> 文件位置：`motor.c`中`Get_Angle`函数 

> 功能：获取角度

```C
/**************************************************************************
Function: Get angle
Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
Output  : none
函数功能：获取角度	
入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		if(Gyro_X>32768)  Gyro_X-=65536;                 //数据类型转换  也可通过short强制类型转换
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //数据类型转换  也可通过short强制类型转换
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //数据类型转换
		if(Accel_X>32768) Accel_X-=65536;                //数据类型转换
		if(Accel_Y>32768) Accel_Y-=65536;                //数据类型转换
		if(Accel_Z>32768) Accel_Z-=65536;                //数据类型转换
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //计算倾角，转换单位为度	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //计算倾角，转换单位为度
		Gyro_X=Gyro_X/16.4;                              //陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册
		Gyro_Y=Gyro_Y/16.4;                              //陀螺仪量程转换	
		if(Way_Angle==2)		  	
		{
			 Pitch = -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//卡尔曼滤波
			 Roll = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计	
	}
}
```

这段代码是用于获取角度信息的函数，其主要功能是根据不同的算法（DMP、Kalman、Complementary Filtering）获取倾角信息。

函数的输入参数是 `way`，用于指定获取角度的算法：

- `1` 表示使用 DMP 算法。

- `2` 表示使用 Kalman 算法。

- `3` 表示使用互补滤波（Complementary Filtering）算法。

在函数内部，根据选择的算法，它会从 MPU6050 传感器中读取加速度计和陀螺仪的数据，并计算出倾角、角速度等信息。

具体流程如下：

- 如果 `way` 为 `1`（DMP 算法），则调用 `Read_DMP` 函数来读取加速度、角速度和倾角信息。

- 如果 `way` 不为 `1`，则从 MPU6050 传感器中读取原始的陀螺仪和加速度计数据。

- 对读取到的数据进行处理，包括单位转换、符号调整等。

- 根据不同的算法选择，使用卡尔曼滤波或互补滤波来计算倾角信息，分别存储在 `Pitch` 和 `Roll` 变量中。

- 最后，更新平衡倾角、转向角速度和 Z 轴加速度计数据。

这个函数的目的是获取角度信息，通常在机器人、飞行器等控制系统中用于姿态控制。不同的算法选择会影响获取的角度精度和响应速度。



> 文件位置：`MPU6050.c`中`Read_DMP`函数

> 功能：`way=1`，使用`DMP`算法获取角度（使用`MPU6050`官方的库函数）

```C
/**************************************************************************
Function: Read the attitude information of DMP in mpu6050
Input   : none
Output  : none
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
**************************************************************************/
void Read_DMP(void)
{	
	  unsigned long sensor_timestamp;
		unsigned char more;
		long quat[4];

				dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		//读取DMP数据
				if (sensors & INV_WXYZ_QUAT )
				{    
					 q0=quat[0] / q30;
					 q1=quat[1] / q30;
					 q2=quat[2] / q30;
					 q3=quat[3] / q30; 		//四元数
					 Roll = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//计算出横滚角
					 Pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // 计算出俯仰角
					 Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	 //计算出偏航角
				}

}
```

这段代码是用于读取 MPU6050 内置 DMP 的姿态信息的函数。

函数名：`Read_DMP`，其主要功能如下：

1. 调用 `dmp_read_fifo` 函数（`MPU6050`官方提供的库函数），该函数用于读取 DMP 的数据，包括陀螺仪（gyro）、加速度计（accel）、四元数（quat）等信息。这些数据将被存储在相应的数组中。因为四元数是放大了2^30存储的，所以最终结果要除以2^30。

2. 对读取的四元数数据进行处理，将其转换为横滚角（Roll）、俯仰角（Pitch）和偏航角（Yaw）。

    - 计算横滚角 Roll：通过四元数中的公式计算，得到横滚角。

    - 计算俯仰角 Pitch：同样通过四元数中的公式计算，得到俯仰角。

    - 计算偏航角 Yaw：同样通过四元数中的公式计算，得到偏航角。

这些角度信息通常用于飞行器、机器人等需要姿态控制的应用中。四元数是一种用于表示三维旋转的数学工具，通过计算四元数，可以获取物体的旋转姿态。这段代码利用 MPU6050 的 DMP 功能，读取传感器数据并转换为姿态信息，以实现精确的姿态控制。



## PID控制

> 文件位置：`control.c`中的`EXTI15_10_IRQHandler`函数里调用`平衡PID控制`，`速度环PID控制`，`转向环PID控制`函数

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
      //省略部分代码
      Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		Turn_Pwm=Turn(Gyro_Turn);														//转向环PID控制     
      //省略部分代码
      return 0;
    }
}
```



> 文件位置：`control.c`中的`Balance`函数

> 功能：直立环控制函数

```C
/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制		
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
```

这段代码是用于进行直立 PD（比例-微分）控制的函数，根据角度和角速度来计算直立控制的 PWM（脉冲宽度调制）信号。下面进行详细讲解：

- 函数名：`Balance`

- 输入参数：`Angle` 表示角度，`Gyro` 表示角速度。

- 输出：`balance` 表示直立控制的 PWM 信号。

函数内部实现逻辑如下：

1. `Angle_bias` 计算：首先，通过将平衡中值 `Middle_angle` 零度与当前角度 `Angle` 相减，计算出角度的偏差 `Angle_bias`。这个偏差表示当前角度与平衡位置的差距。

2. `Gyro_bias` 计算：然后，将零点角速度（0-Gyro）与当前角速度 `Gyro` 相减，得到角速度的偏差 `Gyro_bias`。

3. `balance` 计算：接下来，使用 PD 控制算法，通过将 `Angle_bias` 乘以比例系数 `Balance_Kp` 和 `Gyro_bias` 乘以微分系数 `Balance_Kd`，然后将二者相加，计算出直立控制的 PWM 信号 `balance`。PD 控制是一种控制算法，它通过比例项（P项）和微分项（D项）来控制系统。

    - `Balance_Kp`：是比例系数，用于调整比例项的影响。增大 `Balance_Kp` 会增强系统对角度偏差的敏感度。

    - `Balance_Kd`：是微分系数，用于调整微分项的影响。增大 `Balance_Kd` 会增强系统对角速度偏差的敏感度。

    - 这些系数需要根据具体的应用和系统来调整，以实现合适的控制性能。

1. 最后，函数返回 `balance`，这个值可以用于控制电机或其他执行器，以实现直立控制。在具体应用中，可以根据 `balance` 的值来调整电机的 PWM 信号，以使系统保持在平衡状态。

总的来说，这个函数是一个用于直立控制的控制器，根据角度和角速度信息，计算出合适的 PWM 信号，用于维持系统的直立状态。这种控制通常用于平衡车、机器人等系统。



> 文件位置：`control.c`中的`Turn`函数

> 功能：转向环控制函数

```C
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制 
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//修改转向速度，请修改Turn_Amplitude即可
	//===================遥控左右旋转部分=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;        
	 else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  //===================转向PD控制器=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100;//结合Z轴陀螺仪进行PD控制
	 return turn;								 				 //转向环PWM右转为正，左转为负
}
```

这段代码是用于转向控制的函数，根据Z轴陀螺仪的角速度来计算转向控制的PWM信号。下面进行详细讲解：

- 函数名：`Turn`

- 输入参数：`gyro` 表示Z轴陀螺仪的角速度。

- 输出：`turn` 表示转向控制的PWM信号。

函数内部实现逻辑如下：

1. 定义一些局部变量，如`Turn_Target`（转向目标角速度）、`turn`（实际的转向控制PWM信号）和`Turn_Amplitude`（转向幅度）等。

2. 针对遥控左右旋转的情况：根据标志变量`Flag_Left`和`Flag_Right`的状态，设置`Turn_Target`的值。如果`Flag_Left`为1，表示需要左转，那么`Turn_Target`被设置为负的`Turn_Amplitude`除以`Flag_velocity`，如果`Flag_Right`为1，表示需要右转，那么`Turn_Target`被设置为正的`Turn_Amplitude`除以`Flag_velocity`。如果都不是，`Turn_Target`被设置为0。

3. 针对前进或后退的情况：根据标志变量`Flag_front`和`Flag_back`的状态，设置`Kd`的值。如果任何一个标志为1，表示机器处于前进或后退状态，那么`Kd`被设置为`Turn_Kd`，否则设置为0。这里取消了陀螺仪的纠正，具体原因可能是因为在转向的过程中不希望受到陀螺仪的影响。

4. 转向PD控制器：使用PD控制算法，将`Turn_Target`乘以比例系数`Kp`和`gyro`乘以微分系数`Kd`，然后将二者相加，计算出转向控制的PWM信号`turn`。PD控制是一种控制算法，通过比例项（P项）和微分项（D项）来控制系统。

    - `Turn_Kp`：是比例系数，用于调整比例项的影响。增大`Turn_Kp`会增强系统对转向角速度偏差的敏感度。

    - `Turn_Kd`：是微分系数，用于调整微分项的影响。增大`Turn_Kd`会增强系统对转向角速度变化率的敏感度。

1. 最后，函数返回`turn`，这个值可以用于控制电机或其他执行器，以实现转向控制。在具体应用中，可以根据`turn`的值来调整电机的PWM信号，以使系统实现所需的转向行为。

总的来说，这个函数用于根据Z轴陀螺仪的角速度信息来计算转向控制的PWM信号，以实现转向控制。这种控制通常用于机器人等系统，以调整方向或位置。



> 文件位置：`control.c`中的`EXTI15_10_IRQHandler`函数

```C
int EXTI15_10_IRQHandler(void) 
{
  //省略部分代码
  int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//平衡环PWM变量，速度环PWM变量，转向环PWM变
  if(INT==0)		
	{   
      Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //计算左轮电机最终PWM
      Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //计算右轮电机最终PWM
      if(Turn_Off(Angle_Balance,Voltage)==0)     					//如果不存在异常
			Set_Pwm(Motor_Left,Motor_Right);         					//赋值给PWM寄存器  
  
}
```

这段代码是用于计算左右轮电机的最终PWM信号，并将其赋值给PWM寄存器以控制电机的运行。下面进行详细解释：

1. `Motor_Left` 和 `Motor_Right` 是两个变量，分别用于存储左轮和右轮电机的最终PWM信号。

2. `Motor_Left` 的计算：它等于 `Balance_Pwm`（平衡控制的PWM信号） + `Velocity_Pwm`（速度控制的PWM信号） + `Turn_Pwm`（转向控制的PWM信号）。这个公式将平衡、速度和转向控制的PWM信号合并到左轮电机的PWM信号中。

3. `Motor_Right` 的计算：它等于 `Balance_Pwm`（平衡控制的PWM信号） + `Velocity_Pwm`（速度控制的PWM信号） - `Turn_Pwm`（转向控制的PWM信号）。这个公式将平衡、速度和转向控制的PWM信号合并到右轮电机的PWM信号中。

4. 这些计算是基于所述的控制算法的结果，其中包括平衡控制、速度控制和转向控制。这些算法根据传感器数据和系统需求来计算出相应的PWM信号，以控制左右轮电机的运动。

5. 接下来，通过 `if(Turn_Off(Angle_Balance,Voltage)==0)` 来检查是否存在异常。`Turn_Off` 函数可能是用于检测异常情况的函数。如果没有异常，就会执行以下操作。

6. `Set_Pwm(Motor_Left, Motor_Right)` 用于将计算得到的左右轮电机的最终PWM信号赋值给PWM寄存器，以控制电机的速度和方向。

总之，这段代码是电机控制的一部分，通过将不同控制算法的输出相加，计算左右轮电机的最终PWM信号，并将其应用于电机以实现所需的运动。异常检测部分可能用于处理系统异常情况，以确保电机安全运行。



## 蓝牙遥控功能

本节讲解APP蓝牙遥控功能

> 文件位置：`MiniBalance.c`中的`main`函数

```C
int main()
{
    uart3_init(9600);             	//串口3初始化，用于蓝牙模块
    while(1)
	{
		if(Flag_Show==0)          		//使用MiniBalance APP和OLED显示屏
		{
			 APP_Show();								//发送数据给APP
			 oled_show();          			//显示屏打开
		}
    }
}
```

APP使用串口3，波特率是9600。它具有调试，监控，遥控等功能。

使用`APP_Show`函数进行监控。

> 文件位置：`show.c`中的`main`函数

> 功能：向APP发送数据

```C
/**************************************************************************
Function: Send data to APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
  static u8 flag;
	int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
	Voltage_Show=(Voltage-1110)*2/3;		if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
	Encoder_Right_Show=Velocity_Right*1.1; if(Encoder_Right_Show<0) Encoder_Right_Show=-Encoder_Right_Show;			  //对编码器数据就行数据处理便于图形化
	Encoder_Left_Show=Velocity_Left*1.1;  if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
	flag=!flag;
	if(PID_Send==1)			//发送PID参数,在APP调参界面显示
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Balance_Kp,(int)Balance_Kd,(int)Velocity_Kp,(int)Velocity_Ki,(int)Turn_Kp,(int)Turn_Kd,0,0,0);//打印到APP上面	
		PID_Send=0;	
	}	
   else	if(flag==0)		// 发送电池电压，速度，角度等参数，在APP首页显示
		printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)Angle_Balance); //打印到APP上面
	 else								//发送小车姿态角，在波形界面显示
	  printf("{B%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw); //x，y，z轴角度 在APP上面显示波形
																													//可按格式自行增加显示波形，最多可显示五个
}
```

这段代码是用于向一个APP发送数据的函数。以下是关于这个函数的详细解释：

1. `APP_Show` 函数用于向一个APP发送数据，这些数据将在APP上显示或用于其他处理。

2. 在函数内部，有一些静态变量和局部变量，用于存储要发送的数据。这些数据包括电压、编码器数据、PID参数、姿态角等。

3. `Voltage_Show` 变量计算了电池电压的显示值。它通过对电压数据进行处理，将原始数据映射到一个合适的范围内，并进行了一些限制。

4. `Encoder_Left_Show` 和 `Encoder_Right_Show` 变量对左右电机的速度进行了处理，以便在图形化界面上显示。它们确保速度值为正数。

5. `flag` 变量用于控制数据发送的频率。在每次函数调用时，它会切换状态，以便在一次调用中发送不同的数据。

6. 函数的逻辑分为两个部分：

    - 如果 `PID_Send` 变量的值为1，说明需要发送PID参数。在这种情况下，函数会将PID参数打印到APP上，以便在APP的调参界面显示。然后，将 `PID_Send` 设为0，表示已发送PID参数。

    - 如果 `PID_Send` 不为1，且 `flag` 的值为0，说明需要发送电池电压、速度、角度等参数。在这种情况下，函数会将这些参数打印到APP上，以在APP的首页显示。

1. 打印的数据以一定的格式发送到APP，数据包括左右电机速度、电池电压、角度等信息。

总之，这个函数是用于将各种参数和数据发送到APP的，以便在APP上进行显示或其他用途。它根据不同的条件和标志选择性地发送不同的数据。



当主控板收到app发来数据的时候，我们就会进入中断函数进行处理。

> 文件位置：`usart3.c`中的`USART3_IRQHandler`函数

> 功能：串口3接收中断，处理app发来的数据

```C
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


```

这是一个用于处理串口3接收中断的函数。以下是对该函数的详细解释：

1. `USART3_IRQHandler` 函数是串口3接收中断的处理函数。当串口3接收到数据时，会触发这个中断。

2. 首先，函数检查是否接收到数据，通过以下代码行：

    ```C
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    ```

    如果接收到数据，将执行接下来的操作。

1. 函数内有一些静态变量和局部变量，用于存储和处理接收到的数据。例如，`uart_receive` 用于存储接收到的数据，`Flag_PID` 用于标识是否接收到了APP的参数指令，`i` 和 `j` 用于数据接收计数，`Receive` 数组用于存储接收到的数据，`Data` 用于存储解析后的数值。

2. 函数会根据接收到的数据做不同的处理：

    - 如果接收到 `0x59`，则将 `Flag_velocity` 设置为2，表示低速挡（默认值）。

    - 如果接收到 `0x58`，则将 `Flag_velocity` 设置为1，表示高速挡。

    - 如果接收到 `uart_receive` 大于10，表示接收到控制指令，会设置不同的标志位，如 `Flag_front`、`Flag_back`、`Flag_Left`、`Flag_Right` 用于控制小车的前进、后退、左转、右转等动作。

    - 如果接收到 `0x7B`，则将 `Flag_PID` 设置为1，表示开始采集参数指令。

    - 如果接收到 `0x7D`，则将 `Flag_PID` 设置为2，表示停止采集参数指令，并分析接收到的参数数据。

    ![5.png](/static/README-image/5.png)

    ![6.png](/static/README-image/6.png)

3. 如果 `Flag_PID` 为1，表示正在采集数据，将接收到的数据存储在 `Receive` 数组中，并增加计数器 `i`。

4. 如果 `Flag_PID` 为2，表示停止采集数据，并对接收到的参数数据进行解析。根据接收到的数据内容，设置对应的参数，如 `Balance_Kp`、`Balance_Kd`、`Velocity_Kp`、`Velocity_Ki`、`Turn_Kp`、`Turn_Kd`。

5. 最后，清空相关的标志位、计数器和数据缓冲区，以准备接收下一组参数或控制指令。

总之，这个函数用于处理串口3接收到的数据，包括控制指令和参数设置指令，根据不同的指令对小车进行控制或更新控制参数。它允许通过串口与小车进行交互和控制。



遥控前进后退的功能是在速度环里面实现的。

> 文件位置：`control.c`中`Velocity`函数

> 功能：速度控制pwm

```C
/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM		
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
//修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;
	  //================遥控前进后退部分====================// 
		if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 30; //如果进入跟随/避障模式,降低速度
		else 											        Target_Velocity = 50;
		if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //收到前进信号
		else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //收到后退信号
	  else  Movement=0;	
	
   //=============超声波功能（跟随/避障）==================// 
	  if(Flag_follow==1&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //跟随
			 Movement=Target_Velocity/Flag_velocity;
		if(Flag_follow==1&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
			 Movement=-Target_Velocity/Flag_velocity;
		if(Flag_avoid==1&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //超声波避障
			 Movement=-Target_Velocity/Flag_velocity;
		
   //================速度PI控制器=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和） 
		Encoder_bias *= 0.84;		                                          //一阶低通滤波器       
		Encoder_bias += Encoder_Least*0.16;	                              //一阶低通滤波器，减缓速度变化 
		Encoder_Integral +=Encoder_bias;                                  //积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //积分限幅
		if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //积分限幅	
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //速度控制	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//电机关闭后清除积分
	  return velocity;
}
```

这是一个用于执行速度控制的函数，其中使用了PI（比例-积分）控制器来控制小车的速度。以下是对该函数的详细解释：

1. `Velocity` 函数的主要功能是根据左右轮编码器的读数来控制小车的速度，生成相应的速度控制PWM信号。

2. 首先，函数内部定义了一些静态变量，用于存储速度相关的信息，如 `velocity` 用于存储最终的速度控制PWM，`Encoder_Least` 用于存储速度偏差，`Encoder_bias` 和 `Encoder_Integral` 分别用于一阶低通滤波和积分项。

3. 根据遥控器的控制信号和标志位，决定小车是前进、后退还是停止。这部分的逻辑通过以下代码实现：

    ```C
    if (Flag_follow == 1 || Flag_avoid == 1) Target_Velocity = 30; // 如果进入跟随/避障模式,降低速度
    else Target_Velocity = 50;
    if (Flag_front == 1) Movement = Target_Velocity / Flag_velocity;     // 收到前进信号
    else if (Flag_back == 1) Movement = -Target_Velocity / Flag_velocity; // 收到后退信号
    else Movement = 0;
    ```

    在跟随模式和避障模式下，降低了目标速度以适应特殊情况。`Flag_front` 和 `Flag_back` 标志位用于检测是否接收到前进和后退的信号。

1. 在速度PI控制器部分，首先计算最新的速度偏差 `Encoder_Least`，它等于目标速度（这里设定为零）减去测量速度（左右编码器的读数之和）。

2. 接下来，通过一阶低通滤波器对速度偏差进行滤波，以减缓速度变化。

3. 使用积分项 `Encoder_Integral` 来积分速度偏差，这是为了处理静态误差，积分时间为10ms。

4. 对积分项进行限幅，避免积分项过大。

5. 最终，根据速度偏差和积分项，计算出速度控制的PWM信号 `velocity`。这是一个基于PI控制的速度控制器，其中 `Velocity_Kp` 和 `Velocity_Ki` 是比例和积分系数，它们用于调节速度控制的响应特性。

6. 最后，如果电机关闭（通过 `Turn_Off` 函数检测），或者标志位 `Flag_Stop` 被设置为1，将清零积分项，以防止不必要的积分。

总之，这个函数用于根据编码器读数执行速度控制，可以在不同的模式下控制小车前进、后退、停止，并通过PI控制器来维持目标速度。



转向在转向盘里面实现

> 文件位置：`control.c`中`Turn`函数

> 功能：转向控制

```C
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制 
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
作    者：轮趣科技（东莞）有限公司
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//修改转向速度，请修改Turn_Amplitude即可
	//===================遥控左右旋转部分=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;        
	 else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  //===================转向PD控制器=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100;//结合Z轴陀螺仪进行PD控制
	 return turn;								 				 //转向环PWM右转为正，左转为负
}

```



## 超声波测距



## 感谢

部分开源资源来自轮趣科技[https://wheeltec.net/](https://wheeltec.net/)



