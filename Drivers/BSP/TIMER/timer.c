#include "timer.h"
#include "./BSP/LED/led.h"

TIM_HandleTypeDef TIM9_Handler;      	//��ʱ����� 
TIM_OC_InitTypeDef TIM9_CH2Handler;	//��ʱ��14ͨ��1���

//TIM9 PWM���ֳ�ʼ�� 
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
void TIM9_PWM_Init(uint16_t arr,uint16_t psc,uint16_t pulse)
{  
    TIM9_Handler.Instance=TIM9;          	//��ʱ��9
    TIM9_Handler.Init.Prescaler=psc;       //��ʱ����Ƶ
    TIM9_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
    TIM9_Handler.Init.Period=arr;          //�Զ���װ��ֵ
    TIM9_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM9_Handler);       //��ʼ��PWM
    
    TIM9_CH2Handler.OCMode=TIM_OCMODE_PWM1; //ģʽѡ��PWM1
    TIM9_CH2Handler.Pulse=pulse;            //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    //TIM9_CH2Handler.OCPolarity=TIM_OCPOLARITY_LOW; //����Ƚϼ���Ϊ�� TIM_OCPOLARITY_HIGH
		TIM9_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH;//����Ƚϼ���Ϊ��
    HAL_TIM_PWM_ConfigChannel(&TIM9_Handler,&TIM9_CH2Handler,TIM_CHANNEL_2);//����TIM9ͨ��2
	
    HAL_TIM_PWM_Start(&TIM9_Handler,TIM_CHANNEL_2);//����PWMͨ��2
}



//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM9_CLK_ENABLE();			//ʹ�ܶ�ʱ��9
	__HAL_RCC_GPIOE_CLK_ENABLE();			//����GPIOEʱ��
	
	GPIO_Initure.Pin=GPIO_PIN_6;           	//PE6
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������
	GPIO_Initure.Pull=GPIO_PULLUP;          //����
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
	GPIO_Initure.Alternate= GPIO_AF3_TIM9;	//PE6����ΪTIM9
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}

//����TIM9ͨ����ռ�ձ�
//compare:�Ƚ�ֵ
void TIM_SetTIM9Compare2(uint32_t compare)
{
	TIM9->CCR2=compare; 
}


