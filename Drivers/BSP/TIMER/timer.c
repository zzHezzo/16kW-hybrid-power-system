#include "timer.h"
#include "./BSP/LED/led.h"

TIM_HandleTypeDef TIM9_Handler;      	//定时器句柄 
TIM_OC_InitTypeDef TIM9_CH2Handler;	//定时器14通道1句柄

//TIM9 PWM部分初始化 
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void TIM9_PWM_Init(uint16_t arr,uint16_t psc,uint16_t pulse)
{  
    TIM9_Handler.Instance=TIM9;          	//定时器9
    TIM9_Handler.Init.Prescaler=psc;       //定时器分频
    TIM9_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM9_Handler.Init.Period=arr;          //自动重装载值
    TIM9_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM9_Handler);       //初始化PWM
    
    TIM9_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM9_CH2Handler.Pulse=pulse;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    //TIM9_CH2Handler.OCPolarity=TIM_OCPOLARITY_LOW; //输出比较极性为低 TIM_OCPOLARITY_HIGH
		TIM9_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH;//输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM9_Handler,&TIM9_CH2Handler,TIM_CHANNEL_2);//配置TIM9通道2
	
    HAL_TIM_PWM_Start(&TIM9_Handler,TIM_CHANNEL_2);//开启PWM通道2
}



//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM9_CLK_ENABLE();			//使能定时器9
	__HAL_RCC_GPIOE_CLK_ENABLE();			//开启GPIOE时钟
	
	GPIO_Initure.Pin=GPIO_PIN_6;           	//PE6
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF3_TIM9;	//PE6复用为TIM9
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}

//设置TIM9通道的占空比
//compare:比较值
void TIM_SetTIM9Compare2(uint32_t compare)
{
	TIM9->CCR2=compare; 
}


