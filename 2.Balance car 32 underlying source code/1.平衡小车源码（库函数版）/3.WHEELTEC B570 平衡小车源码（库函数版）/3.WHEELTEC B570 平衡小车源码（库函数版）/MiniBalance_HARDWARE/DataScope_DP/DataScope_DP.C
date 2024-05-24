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
#include "DataScope_DP.h"
unsigned char DataScope_OutPut_Buffer[42] = {0};	   //串口发送缓冲区
/**************************************************************************
Function: Convert single precision floating-point data into 4-byte data and store it in the specified address
Input   : target:Target single precision data  buf:Array to be written  beg:Specifies which element of the array to write from
Output  : none
函数功能：将单精度浮点数据转成4字节数据并存入指定地址  附加说明：用户无需直接操作此函数
入口参数：target:目标单精度数据  buf:待写入数组  beg:指定从数组第几个元素开始写入
返回  值：无
**************************************************************************/
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
/**************************************************************************
Function: Write the single precision floating-point data of the channel to be sent to the send buffer
Input   : Data：Channel data  Channel：Select channel (1-10)
Output  : none
函数功能：将待发送通道的单精度浮点数据写入发送缓冲区
入口参数：Data：通道数据  Channel：选择通道（1-10）
返回  值：无
**************************************************************************/
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}
/**************************************************************************
Function: Generate frame format that datascope v1.0 can recognize correctly
Input   : Channel_Number：Number of channels to send
Output  : Returns the number of send buffer data
函数功能：生成 DataScopeV1.0 能正确识别的帧格式
入口参数：Channel_Number：需要发送的通道个数
返回  值：返回发送缓冲区数据个数   返回0表示帧格式生成失败
**************************************************************************/
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';  //帧头
		
	 switch(Channel_Number)   
   { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;  
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; 
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;  
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; 
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; 
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; 
   }	 
  }
	return 0;
}

