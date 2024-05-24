/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��5.7
�޸�ʱ�䣺2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update��2021-04-29

All rights reserved
***********************************************/
#include "control.h"	
/**************************************************************************
Function: Control function
Input   : none
Output  : none
�������ܣ����еĿ��ƴ��붼��������
         5ms�ⲿ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��	
��ڲ�������
����  ֵ����				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//��ѹ������ر���
	static u8 Flag_Target;																//���ƺ�����ر������ṩ10ms��׼
	int Encoder_Left,Encoder_Right;             					//���ұ��������������
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;		  					//ƽ�⻷PWM�������ٶȻ�PWM������ת��PWM��
	if(INT==0)		
	{   
		EXTI->PR=1<<12;                           					//����жϱ�־λ   
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
		Encoder_Left=-Read_Encoder(2);            					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		Encoder_Right=-Read_Encoder(4);           					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
																												//����A���TIM2_CH1,����A���TIM4_CH2,�����������������ļ�����ͬ
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);//����������ת�ٶȣ�mm/s��
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;  		//���������ṩ50ms�ľ�׼��ʱ��ʾ������Ҫ50ms�߾�����ʱ
		}
		if(Flag_Target==1)                        					//10ms����һ��
		{
			Voltage_Temp=Get_battery_volt();		    					//��ȡ��ص�ѹ		
			Voltage_Count++;                       						//ƽ��ֵ������
			Voltage_All+=Voltage_Temp;              					//��β����ۻ�
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ		
			return 0;	                                               
		}                                         					//10ms����һ��
		Read_Distane();                           					//��ȡ��������������ֵ
		if(Flag_follow==0&&Flag_avoid==0)	Led_Flash(100);   //LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
		if(Flag_follow==1||Flag_avoid==1)	Led_Flash(0);     //LED����;����������/����ģʽ	
		Key();                                    					//ɨ�谴��״̬ ����˫�����Ըı�С������״̬
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //ƽ��PID���� Gyro_Balanceƽ����ٶȼ��ԣ�ǰ��Ϊ��������Ϊ��
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //�ٶȻ�PID����	��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		Turn_Pwm=Turn(Gyro_Turn);														//ת��PID����     
		
		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //�������ֵ������PWM
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //�������ֵ������PWM
																												//PWMֵ����ʹС��ǰ��������ʹС������
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM�޷�
		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Flag_Stop=1;	                           					//���������͹رյ��
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//����Ƿ�С��������
			Flag_Stop=0;	                           					//��������¾��������
		Choose(Encoder_Left,Encoder_Right);									//ת������ѡ��С��ģʽ
		if(Turn_Off(Angle_Balance,Voltage)==0)     					//����������쳣
			Set_Pwm(Motor_Left,Motor_Right);         					//��ֵ��PWM�Ĵ���  
	 }       	
	 return 0;	  
} 

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle��Gyro��angular velocity
Output  : balance��Vertical control PWM
�������ܣ�ֱ��PD����		
��ڲ�����Angle:�Ƕȣ�Gyro�����ٶ�
����  ֵ��balance��ֱ������PWM
**************************************************************************/	
int Balance(float Angle,float Gyro)
{  
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;                       				//���ƽ��ĽǶ���ֵ �ͻ�е���
	 Gyro_bias=0-Gyro; 
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; //����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left��Left wheel encoder reading��encoder_right��Right wheel encoder reading
Output  : Speed control PWM
�������ܣ��ٶȿ���PWM		
��ڲ�����encoder_left�����ֱ�����������encoder_right�����ֱ���������
����  ֵ���ٶȿ���PWM
**************************************************************************/
//�޸�ǰ�������ٶȣ����޸�Target_Velocity�����磬�ĳ�60�ͱȽ�����
int Velocity(int encoder_left,int encoder_right)
{  
    static float velocity,Encoder_Least,Encoder_bias,Movement;
	  static float Encoder_Integral,Target_Velocity;
	  //================ң��ǰ�����˲���====================// 
		if(Flag_follow==1||Flag_avoid==1) Target_Velocity = 30; //����������/����ģʽ,�����ٶ�
		else 											        Target_Velocity = 50;
		if(Flag_front==1)    	Movement=Target_Velocity/Flag_velocity;	  //�յ�ǰ���ź�
		else if(Flag_back==1)	Movement=-Target_Velocity/Flag_velocity;  //�յ������ź�
	  else  Movement=0;	
	
   //=============���������ܣ�����/���ϣ�==================// 
	  if(Flag_follow==1&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) //����
			 Movement=Target_Velocity/Flag_velocity;
		if(Flag_follow==1&&Distance<200&&Flag_Left!=1&&Flag_Right!=1) 
			 Movement=-Target_Velocity/Flag_velocity;
		if(Flag_avoid==1&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)  //����������
			 Movement=-Target_Velocity/Flag_velocity;
		
   //================�ٶ�PI������=====================//	
		Encoder_Least =0-(encoder_left+encoder_right);                    //��ȡ�����ٶ�ƫ��=Ŀ���ٶȣ��˴�Ϊ�㣩-�����ٶȣ����ұ�����֮�ͣ� 
		Encoder_bias *= 0.84;		                                          //һ�׵�ͨ�˲���       
		Encoder_bias += Encoder_Least*0.16;	                              //һ�׵�ͨ�˲����������ٶȱ仯 
		Encoder_Integral +=Encoder_bias;                                  //���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral+Movement;                       //����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //�����޷�
		if(Encoder_Integral<-10000)	  Encoder_Integral=-10000;            //�����޷�	
		velocity=-Encoder_bias*Velocity_Kp/100-Encoder_Integral*Velocity_Ki/100;     //�ٶȿ���	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;//����رպ��������
	  return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
�������ܣ�ת����� 
��ڲ�����Z��������
����  ֵ��ת�����PWM
��    �ߣ���Ȥ�Ƽ�����ݸ�����޹�˾
**************************************************************************/
int Turn(float gyro)
{
	 static float Turn_Target,turn,Turn_Amplitude=54;
	 float Kp=Turn_Kp,Kd;			//�޸�ת���ٶȣ����޸�Turn_Amplitude����
	//===================ң��������ת����=================//
	 if(1==Flag_Left)	        Turn_Target=-Turn_Amplitude/Flag_velocity;
	 else if(1==Flag_Right)	  Turn_Target=Turn_Amplitude/Flag_velocity; 
	 else Turn_Target=0;
	 if(1==Flag_front||1==Flag_back)  Kd=Turn_Kd;        
	 else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  //===================ת��PD������=================//
	 turn=Turn_Target*Kp/100+gyro*Kd/100;//���Z�������ǽ���PD����
	 return turn;								 				 //ת��PWM��תΪ������תΪ��
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left��Left wheel PWM��motor_right��Right wheel PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	    BIN1=1,			BIN2=0; //ǰ�� 
	else           			  BIN1=0,			BIN2=1; //����
	PWMB=myabs(motor_left);	
  if(motor_right>0)			AIN2=1,			AIN1=0;	//ǰ��
	else 	        			  AIN2=0,			AIN1=1; //����
	PWMA=myabs(motor_right);
}
/**************************************************************************
Function: PWM limiting range
Input   : IN��Input  max��Maximum value  min��Minimum value
Output  : Output
�������ܣ�����PWM��ֵ 
��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
����  ֵ���޷����ֵ
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); 
	if(tmp==1)
	{ 
		Flag_Stop=!Flag_Stop;
	}		//��������С������ͣ
	tmp2=Long_Press();                   
  if(tmp2==1) Flag_Show=!Flag_Show;	//�������ƽ�����λ��ģʽ��С������ʾֹͣ

}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle��Car inclination��voltage��Voltage
Output  : 1��abnormal��0��normal
�������ܣ��쳣�رյ��		
��ڲ�����angle��С����ǣ�voltage����ѹ
����  ֵ��1���쳣  0������
**************************************************************************/	
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000)//��ص�ѹ����10V�رյ��
	{	                                                 //��Ǵ���40�ȹرյ��
		temp=1;                                          //Flag_Stop��1�����������ƹرյ��
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
	}
	else
		temp=0;
	return temp;			
}
	
/**************************************************************************
Function: Get angle
Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
Output  : none
�������ܣ���ȡ�Ƕ�	
��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/	
void Get_Angle(u8 way)
{ 
	float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
	Temperature=Read_Temperature();      //��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	if(way==1)                           //DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{	
		Read_DMP();                      	 //��ȡ���ٶȡ����ٶȡ����
		Angle_Balance=Pitch;             	 //����ƽ�����,ǰ��Ϊ��������Ϊ��
		Gyro_Balance=gyro[0];              //����ƽ����ٶ�,ǰ��Ϊ��������Ϊ��
		Gyro_Turn=gyro[2];                 //����ת����ٶ�
		Acceleration_Z=accel[2];           //����Z����ٶȼ�
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		if(Gyro_X>32768)  Gyro_X-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Y>32768)  Gyro_Y-=65536;                 //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Z>32768)  Gyro_Z-=65536;                 //��������ת��
		if(Accel_X>32768) Accel_X-=65536;                //��������ת��
		if(Accel_Y>32768) Accel_Y-=65536;                //��������ת��
		if(Accel_Z>32768) Accel_Z-=65536;                //��������ת��
		Gyro_Balance=-Gyro_X;                            //����ƽ����ٶ�
		Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��	
		Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;     //������ǣ�ת����λΪ��
		Gyro_X=Gyro_X/16.4;                              //����������ת�������̡�2000��/s��Ӧ������16.4���ɲ��ֲ�
		Gyro_Y=Gyro_Y/16.4;                              //����������ת��	
		if(Way_Angle==2)		  	
		{
			 Pitch = -Kalman_Filter_x(Accel_Angle_x,Gyro_X);//�������˲�
			 Roll = -Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,Gyro_X);//�����˲�
			 Roll = -Complementary_Filter_y(Accel_Angle_y,Gyro_Y);
		}
		Angle_Balance=Pitch;                              //����ƽ�����
		Gyro_Turn=Gyro_Z;                                 //����ת����ٶ�
		Acceleration_Z=Accel_Z;                           //����Z����ٶȼ�	
	}
}
/**************************************************************************
Function: Absolute value function
Input   : a��Number to be converted
Output  : unsigned int
�������ܣ�����ֵ����
��ڲ�����a����Ҫ�������ֵ����
����  ֵ���޷�������
**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration��Z-axis acceleration��Angle��The angle of balance��encoder_left��Left encoder count��encoder_right��Right encoder count
Output  : 1��picked up  0��No action
�������ܣ����С���Ƿ�����
��ڲ�����Acceleration��z����ٶȣ�Angle��ƽ��ĽǶȣ�encoder_left���������������encoder_right���ұ���������
����  ֵ��1:С��������  0��С��δ������
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //��һ��
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //����1��С���ӽ���ֹ
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //����ڶ���
	 {
			if(++count1>200)       count1=0,flag=0;                       //��ʱ���ٵȴ�2000ms�����ص�һ��
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //����2��С������0�ȸ���������
			flag=2; 
	 } 
	 if(flag==2)                                                       //������
	 {
		  if(++count2>100)       count2=0,flag=0;                        //��ʱ���ٵȴ�1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //����3��С������̥��Ϊ�������ﵽ����ת��   
      {
				flag=0;                                                                                     
				return 1;                                                    //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance��Left encoder count��Right encoder count
Output  : 1��put down  0��No action
�������ܣ����С���Ƿ񱻷���
��ڲ�����ƽ��Ƕȣ���������������ұ���������
����  ֵ��1��С������   0��С��δ����
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //��ֹ���      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //����1��С������0�ȸ�����
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //��ʱ���ٵȴ� 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
      {
				flag=0;
				flag=0;
				return 1;                         //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
�������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{ 	
	float Rotation_Speed_L,Rotation_Speed_R;						//���ת��  ת��=������������5msÿ�Σ�*��ȡƵ��/��Ƶ��/���ٱ�/����������
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//����������ٶ�=ת��*�ܳ�
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_left��Left wheel encoder reading��encoder_right��Right wheel encoder reading
Output  : none
�������ܣ�ѡ��С������ģʽ
��ڲ�����encoder_left�������������  encoder_right���ұ���������
����  ֵ����
**************************************************************************/
void Choose(int encoder_left,int encoder_right)
{
	static int count;
	if(Flag_Stop==0)
		count = 0;
	if((Flag_Stop==1)&&(encoder_left<10))	//��ʱֹͣ�����ֲ���
	{
		count += myabs(encoder_right);
		if(count>6&&count<180)	//��ͨģʽ
		{
			Flag_follow = 0;
			Flag_avoid = 0;
		}
		if(count>180&&count<360)	//����ģʽ 
		{
			Flag_avoid = 1;
			Flag_follow = 0;
		}
		if(count>360&&count<540)	//����ģʽ
		{
			Flag_avoid = 0;
			Flag_follow = 1;
		}
		if(count>540)
			count = 0;
	}
}

