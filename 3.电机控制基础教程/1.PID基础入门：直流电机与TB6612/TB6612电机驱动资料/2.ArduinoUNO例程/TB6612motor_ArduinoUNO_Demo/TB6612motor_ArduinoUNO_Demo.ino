
///////////////////TB6612引脚接线///////////////////////////
//直流电机----------TB6612丝印标识----------ArduinoUNO主板引脚
//                     PWMA-----------------3
//                     AIN2-----------------4
//                     AIN1-----------------2
//                     STBY-----------------7
//                     BIN1-----------------8
//                     BIN2-----------------12
//                     PWMB-----------------9
//                     GND------------------GND
//                     VM-------------------12V电池
//                     VCC------------------5V
//                     GND------------------GND
//电机A正极-------------AO1
//电机A负极-------------A02
//电机B负极-------------BO2
//电机B正极-------------BO1
//                     GND-----------------GND
//直流电机----------TB6612丝印标识----------ArduinoUNO主板引脚

//定义引脚名称
#define PWMA 3  //3为模拟引脚，用于PWM控制
#define AIN1 2
#define AIN2 4
#define PWMB 9  //9为模拟引脚，用于PWM控制
#define BIN1 8
#define BIN2 12
#define STBY 7  //2、4、8、12、7为数字引脚，用于开关量控制

int PwmA, PwmB;

void setup() {
  //TB6612电机驱动模块控制信号初始化
  pinMode(AIN1, OUTPUT);//控制电机A的方向，(AIN1, AIN2)=(1, 0)为正转，(AIN1, AIN2)=(0, 1)为反转
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);//控制电机B的方向，(BIN1, BIN2)=(0, 1)为正转，(BIN1, BIN2)=(1, 0)为反转
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);//A电机PWM
  pinMode(PWMB, OUTPUT);//B电机PWM
  pinMode(STBY, OUTPUT);//TB6612FNG使能, 置0则所有电机停止, 置1才允许控制电机

  //初始化TB6612电机驱动模块
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

}

/**************************************************************************
函数功能：设置指定电机转速
入口参数：指定电机motor，motor=1（2）代表电机A（B）； 指定转速pwm，大小范围为0~255，代表停转和满速
返回  值：无
**************************************************************************/
void SetPWM(int motor, int pwm)
{
  if(motor==1&&pwm>=0)//motor=1代表控制电机A，pwm>=0则(AIN1, AIN2)=(1, 0)为正转
  {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, pwm);
  }
  else if(motor==1&&pwm<0)//motor=1代表控制电机A，pwm<0则(AIN1, AIN2)=(0, 1)为反转
  {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, -pwm);
  }
  else if(motor==2&&pwm>=0)//motor=2代表控制电机B，pwm>=0则(BIN1, BIN2)=(0, 1)为正转
  {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, pwm);
  }
  else if(motor==2&&pwm<0)//motor=2代表控制电机B，pwm<0则(BIN1, BIN2)=(1, 0)为反转
  {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, -pwm);
  }
}

void loop() 
{
  SetPWM(1, 255);//电机AB同时满速正转
  SetPWM(2, 255);
  delay(3000);//正转3s
  
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
//  
//  SetPWM(1, 128);//电机AB同时半速正转
//  SetPWM(2, 128);
//  delay(3000);//半速正转3s
//  
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
//  
//  SetPWM(1, -255);//电机AB同时满速反转
//  SetPWM(2, -255);
//  delay(3000);//反转3s
//  
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
//  
//  SetPWM(1, 255);//电机A满速正转
//  SetPWM(2, -255);//电机B满速反转
//  delay(3000);//持续3s
//  
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
}
