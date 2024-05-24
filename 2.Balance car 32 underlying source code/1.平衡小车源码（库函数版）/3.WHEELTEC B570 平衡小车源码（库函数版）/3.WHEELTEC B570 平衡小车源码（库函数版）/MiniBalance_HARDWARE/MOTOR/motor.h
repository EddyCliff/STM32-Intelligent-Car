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
#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMB   TIM1->CCR4  //PA11
#define BIN2   PBout(12)
#define BIN1   PBout(13)
#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define PWMA   TIM1->CCR1  //PA8

void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
