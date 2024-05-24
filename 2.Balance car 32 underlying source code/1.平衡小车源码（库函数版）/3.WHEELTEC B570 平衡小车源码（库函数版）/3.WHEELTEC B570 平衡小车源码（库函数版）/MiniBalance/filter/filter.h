/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：
修改时间：2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:
Update：2021-04-29

All rights reserved
***********************************************/
#ifndef __FILTER_H
#define __FILTER_H
float Kalman_Filter_x(float Accel,float Gyro);		
float Complementary_Filter_x(float angle_m, float gyro_m);
float Kalman_Filter_y(float Accel,float Gyro);		
float Complementary_Filter_y(float angle_m, float gyro_m);

#endif
