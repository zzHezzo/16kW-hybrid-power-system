/**
 ****************************************************************************************************
 * @file        btim.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.1
 * @date        2021-11-29
 * @brief       ������ʱ�� ��������
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
 * V1.0 20211015
 * ��һ�η���
 * V1.1 20211129
 * ��Ӷ�ʱ��3��ʼ���������жϺ���
 ****************************************************************************************************
 */

#include "./BSP/TIMER/btim.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"


extern uint32_t lwip_localtime;         /* lwip����ʱ�������,��λ:ms */

TIM_HandleTypeDef g_tim3_handler;       /* ��ʱ��������� */
TIM_HandleTypeDef g_tim6_handler;       /* ��ʱ��������� */

/**
 * @brief       ������ʱ��TIMX�жϷ�����
 * @param       ��
 * @retval      ��
 */
void BTIM_TIM3_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_tim3_handler);  /* ��ʱ���ص����� */
}

void BTIM_TIM6_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_tim6_handler);  /* ��ʱ���ص����� */
}

/**
 * @brief       �ص���������ʱ���жϷ���������
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&g_tim3_handler))              /* ��ʱ��3 */
    {
        lwip_localtime += 1;

    }
    else if(htim==(&g_tim6_handler))            /* ��ʱ��6 */
    {
        LED1_TOGGLE();
    }
}

/**
 * @brief       ������ʱ��TIMX��ʱ�жϳ�ʼ������
 * @note
 *              ������ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ������ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 84Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void btim_tim3_int_init(uint16_t arr, uint16_t psc)
{
    g_tim3_handler.Instance = BTIM_TIM3_INT;                      /* ͨ�ö�ʱ��X */
    g_tim3_handler.Init.Prescaler = psc;                          /* ����Ԥ��Ƶ��  */
    g_tim3_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* ���ϼ����� */
    g_tim3_handler.Init.Period = arr;                             /* �Զ�װ��ֵ */
    g_tim3_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* ʱ�ӷ�Ƶ���� */
    HAL_TIM_Base_Init(&g_tim3_handler);

    HAL_TIM_Base_Start_IT(&g_tim3_handler);                       /* ʹ��ͨ�ö�ʱ��x�ͼ�������жϣ�TIM_IT_UPDATE */
}

/**
 * @brief       ������ʱ��TIMX��ʱ�жϳ�ʼ������
 * @note
 *              ������ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ������ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ36M, ���Զ�ʱ��ʱ�� = 72Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void btim_tim6_int_init(uint16_t arr, uint16_t psc)
{
    g_tim6_handler.Instance = BTIM_TIM6_INT;                      /* ͨ�ö�ʱ��X */
    g_tim6_handler.Init.Prescaler = psc;                          /* ����Ԥ��Ƶ��  */
    g_tim6_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* ���ϼ����� */
    g_tim6_handler.Init.Period = arr;                             /* �Զ�װ��ֵ */
    g_tim6_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* ʱ�ӷ�Ƶ���� */
    HAL_TIM_Base_Init(&g_tim6_handler);

    HAL_TIM_Base_Start_IT(&g_tim6_handler);                       /* ʹ��ͨ�ö�ʱ��x�ͼ�������жϣ�TIM_IT_UPDATE */
}

/**
 * @brief       ��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
                �˺����ᱻHAL_TIM_Base_Init()��������
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIM3_INT)
    {
        BTIM_TIM3_INT_CLK_ENABLE();                     /* ʹ��TIMʱ�� */
        HAL_NVIC_SetPriority(BTIM_TIM3_INT_IRQn, 1, 3); /* ��ռ1�������ȼ�3����2 */
        HAL_NVIC_EnableIRQ(BTIM_TIM3_INT_IRQn);         /* ����ITM3�ж� */
    }
    if (htim->Instance == BTIM_TIM6_INT)
    {
        BTIM_TIM6_INT_CLK_ENABLE();                     /* ʹ��TIMʱ�� */
        HAL_NVIC_SetPriority(BTIM_TIM6_INT_IRQn, 0, 3); /* ��ռ1�������ȼ�3����2 */
        HAL_NVIC_EnableIRQ(BTIM_TIM6_INT_IRQn);         /* ����ITM3�ж� */
    }
}

