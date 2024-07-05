/**
 ****************************************************************************************************
 * @file        led.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       LED ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */
 
 #include "./BSP/LED/led.h"

/**
 * @brief       ��ʼ��LED���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    LED0_GPIO_CLK_ENABLE();                                 /* LED0ʱ��ʹ�� */
    LED1_GPIO_CLK_ENABLE();                                 /* LED1ʱ��ʹ�� */
    engine_relay_GPIO_CLK_ENABLE();                                 
    motor_relay_GPIO_CLK_ENABLE();                                 
    rectifier_relay_GPIO_CLK_ENABLE();                                 

    gpio_init_struct.Pin = LED0_GPIO_PIN;                   /* LED0���� */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */  
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ���� */
    HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);       /* ��ʼ��LED0���� */

    gpio_init_struct.Pin = LED1_GPIO_PIN;                   /* LED1���� */
    HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);       /* ��ʼ��LED1���� */
    
		gpio_init_struct.Pull = GPIO_NOPULL;                    /* ������������ */
    gpio_init_struct.Pin = engine_relay_GPIO_PIN;                   /* ��Դ�̵������� */
    HAL_GPIO_Init(engine_relay_GPIO_PORT, &gpio_init_struct);       /* ��ʼ����Դ�̵������� */

    gpio_init_struct.Pin = motor_relay_GPIO_PIN;                   /* ����̵������� */
    HAL_GPIO_Init(motor_relay_GPIO_PORT, &gpio_init_struct);       /* ��ʼ������̵������� */

    gpio_init_struct.Pin = rectifier_relay_GPIO_PIN;                   /* ���������� */
    HAL_GPIO_Init(rectifier_relay_GPIO_PORT, &gpio_init_struct);       /* ��ʼ������������ */

	LED0(1);                                                /* �ر� LED0 */
    LED1(1);                                                /* �ر� LED1 */
		//�ر����м̵���
	Engine_relay(1);
	Motor_relay(1);
	Rectifier_relay(1);
}

