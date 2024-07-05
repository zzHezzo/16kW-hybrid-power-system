/**
 ****************************************************************************************************
 * @file        main.c
 * @author      祝智皓
 * @version     V1.0
 * @date        2024-04-23
 * @brief       16kW混合动力系统下位机实时操作系统程序
 * @license     Copyright (c) 2024-2034, 厦门大学
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
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
	
		HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */
    delay_init(168);                    /* 延时初始化 */
    usart_init(115200);                 /* 串口初始化为115200 */
    usmart_dev.init(84);                /* 初始化USMART */
    led_init();                         /* 初始化LED及所有继电器 */
    lcd_init();                         /* 初始化LCD */
//    key_init();                         /* 初始化按键 */
    sram_init();                        /* SRAM初始化 */
//																			//84M/168=0.5M的计数频率 自动重装载为2000，PWM频率为0.5M/2000=250HZ   4ms周期 250Hz 1M
//    TIM9_PWM_Init(2000-1,168-1,1750-1);    	//84M/84=1M的计数频率，自动重装载为500，那么PWM频率为1M/500=2kHZ 
		
    my_mem_init(SRAMIN);                /* 初始化内部SRAM内存池 */
    my_mem_init(SRAMEX);                /* 初始化外部SRAM内存池 */
    my_mem_init(SRAMCCM);               /* 初始化内部CCM内存池 */

    freertos_demo();                    /* 创建系统任务函数 */
}
