/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ף���
 * @version     V1.0
 * @date        2024-04-23
 * @brief       16kW��϶���ϵͳ��λ��ʵʱ����ϵͳ����
 * @license     Copyright (c) 2024-2034, ���Ŵ�ѧ
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 *
 ****************************************************************************************************
 **/


#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./USMART/usmart.h"
#include "./BSP/SRAM/sram.h"
#include "./MALLOC/malloc.h"
#include "freertos_demo.h"
#include "socket_tcp.h"
#include "timer.h"
void zzh_FreeRTOS_sys_run(void);
int main(void)
{
    
		zzh_FreeRTOS_sys_run();
	
}










void zzh_FreeRTOS_sys_run(void)
{
	
		HAL_Init();                         /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7); /* ����ʱ��,168Mhz */
    delay_init(168);                    /* ��ʱ��ʼ�� */
    usart_init(115200);                 /* ���ڳ�ʼ��Ϊ115200 */
    usmart_dev.init(84);                /* ��ʼ��USMART */
    led_init();                         /* ��ʼ��LED�����м̵��� */
    lcd_init();                         /* ��ʼ��LCD */
//    key_init();                         /* ��ʼ������ */
    sram_init();                        /* SRAM��ʼ�� */
//																			//84M/168=0.5M�ļ���Ƶ�� �Զ���װ��Ϊ2000��PWMƵ��Ϊ0.5M/2000=250HZ   4ms���� 250Hz 1M
//    TIM9_PWM_Init(2000-1,168-1,1750-1);    	//84M/84=1M�ļ���Ƶ�ʣ��Զ���װ��Ϊ500����ôPWMƵ��Ϊ1M/500=2kHZ 
		
    my_mem_init(SRAMIN);                /* ��ʼ���ڲ�SRAM�ڴ�� */
    my_mem_init(SRAMEX);                /* ��ʼ���ⲿSRAM�ڴ�� */
    my_mem_init(SRAMCCM);               /* ��ʼ���ڲ�CCM�ڴ�� */

    freertos_demo();                    /* ����ϵͳ������ */
}
