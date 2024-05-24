#include "key.h"
/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
�������ܣ�������ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��PA�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	            //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA 
} 
/**************************************************************************
Function: Key scan
Input   : Double click the waiting time
Output  : 0��No action��1��click��2��Double click
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ������״̬ 0���޶��� 1������ 2��˫�� 
**************************************************************************/
u8 click_N_Double (u8 time)
{
	static	u8 flag_key,count_key,double_key;	
	static	u16 count_single,Forever_count;
	if(KEY==0)  Forever_count++;   //������־λδ��1
	else        Forever_count=0;
	if(0==KEY&&0==flag_key)		flag_key=1;	//��һ�ΰ���
	if(0==count_key)
	{
			if(flag_key==1) 
			{
				double_key++;
				count_key=1;			//��ǰ���һ��
			}
			if(double_key==2) 
			{										//��������
				double_key=0;
				count_single=0;
				return 2;					//˫��ִ�е�ָ��
			}
	}
	if(1==KEY)			flag_key=0,count_key=0;
	
	if(1==double_key)
	{
		count_single++;
		if(count_single>time&&Forever_count<time)
		{
			double_key=0;
			count_single=0;	//��ʱ�����Ϊ˫��
		return 1;//����ִ�е�ָ��
		}
		if(Forever_count>time)
		{
			double_key=0;
			count_single=0;	
		}
	}	
	return 0;
}
/**************************************************************************
Function: Key scan
Input   : none
Output  : 0��No action��1��click��
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������ 
**************************************************************************/
u8 click(void)
{
	static u8 flag_key=1;//�������ɿ���־
	if(flag_key&&KEY==0) //��⵽��������
	{
		flag_key=0;
		return 1;	//��������
	}
	else if(1==KEY)			flag_key=1;
	return 0;//�ް�������
}
/**************************************************************************
Function: Long press detection
Input   : none
Output  : 0��No action��1��Long press for 2 seconds��
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������2s
**************************************************************************/
u8 Long_Press(void)
{
		static u16 Long_Press_count,Long_Press;
	  if(Long_Press==0&&KEY==0)  Long_Press_count++;   //������־λδ��1
    else                       Long_Press_count=0; 
		if(Long_Press_count>200)		//10msɨ��һ��
	  {
			Long_Press=1;	
			Long_Press_count=0;
			return 1;
		}				
		if(Long_Press==1)     //������־λ��1
		{
			Long_Press=0;
		}
		return 0;
}
