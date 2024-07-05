/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       ���ڳ�ʼ������(һ���Ǵ���1)��֧��printf
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

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "zzh.h"
#include "freertos_demo.h"


//extern volatile uint16_t 
//	engine_working_state, engine_speed, engine_warm_up, engine_idling_set,
//	/*		10					         12				     18				          24	*/
//	air_inflow, throttle_position;
//	/*	28				     74		*/
/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");          /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);               /* �ȴ���һ���ַ�������� */

    USART1->DR = (uint8_t)ch;                       /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}
#endif
/***********************************************END*******************************************/



#if USART_EN_RX                                     /* ���ʹ���˽��� */

/* ���ջ���, ���USART_REC_LEN���ֽ�. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/
uint16_t g_usart_rx_sta = 0;

uint16_t g_usart3_rx_sta = 0;
uint8_t RS232_RX_CNT=0; 


	
uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* HAL��ʹ�õĴ��ڽ��ջ��� */

UART_HandleTypeDef g_uart1_handle;                  /* UART��� */


/**
 * @brief       ����X��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              �����USART��ʱ��Դ��sys_stm32_clock_init()�������Ѿ����ù���.
 * @retval      ��
 */
void usart_init(uint32_t baudrate)
{
    g_uart1_handle.Instance = USART_UX;                         /* USART1 */
    g_uart1_handle.Init.BaudRate = baudrate;                    /* ������ */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* �ֳ�Ϊ8λ���ݸ�ʽ */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;             /* һ��ֹͣλ */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;              /* ����żУ��λ */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* ��Ӳ������ */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;                 /* �շ�ģʽ */
    HAL_UART_Init(&g_uart1_handle);                             /* HAL_UART_Init()��ʹ��UART1 */
    
    /* �ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ��������������� */
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief       UART�ײ��ʼ������
 * @param       huart: UART�������ָ��
 * @note        �˺����ᱻHAL_UART_Init()����
 *              ���ʱ��ʹ�ܣ��������ã��ж�����
 * @retval      ��
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    if(huart->Instance == USART_UX)                             /* ����Ǵ���1�����д���1 MSP��ʼ�� */
    {
        USART_UX_CLK_ENABLE();                                  /* USART1 ʱ��ʹ�� */
        USART_TX_GPIO_CLK_ENABLE();                             /* ��������ʱ��ʹ�� */
        USART_RX_GPIO_CLK_ENABLE();                             /* ��������ʱ��ʹ�� */

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;               /* TX���� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* ����������� */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ���� */
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;          /* ����ΪUSART1 */
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);   /* ��ʼ���������� */

        gpio_init_struct.Pin = USART_RX_GPIO_PIN;               /* RX���� */
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;          /* ����ΪUSART1 */
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);   /* ��ʼ���������� */

#if USART_EN_RX
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                      /* ʹ��USART1�ж�ͨ�� */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 0);              /* ��ռ���ȼ�3�������ȼ�0 */
#endif
			
			
		if(huart->Instance == USART3_UX)                             /* ����Ǵ���3�����д���3 MSP��ʼ�� */
    {
        USART3_UX_CLK_ENABLE();                                  /* USART3 ʱ��ʹ�� */
        USART3_TX_GPIO_CLK_ENABLE();                             /* ��������ʱ��ʹ�� */
        USART3_RX_GPIO_CLK_ENABLE();                             /* ��������ʱ��ʹ�� */

        gpio_init_struct.Pin = USART3_TX_GPIO_PIN;               /* TX���� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* ����������� */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ���� */
        gpio_init_struct.Alternate = USART3_TX_GPIO_AF;          /* ����ΪUSART3 */
        HAL_GPIO_Init(USART3_TX_GPIO_PORT, &gpio_init_struct);   /* ��ʼ���������� */

        gpio_init_struct.Pin = USART3_RX_GPIO_PIN;               /* RX���� */
        gpio_init_struct.Alternate = USART3_RX_GPIO_AF;          /* ����ΪUSART3 */
        HAL_GPIO_Init(USART3_RX_GPIO_PORT, &gpio_init_struct);   /* ��ʼ���������� */

#if USART3_EN_RX
        HAL_NVIC_EnableIRQ(USART3_UX_IRQn);                      /* ʹ��USART3�ж�ͨ�� */
        HAL_NVIC_SetPriority(USART3_UX_IRQn, 7, 0);              /* ��ռ���ȼ�7�������ȼ�0 */
#endif
    }
    }
}

/**
 * @brief       Rx����ص�����
 * @param       huart: UART�������ָ��
 * @retval      ��
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART_UX)             /* ����Ǵ���1 */
    {
        if((g_usart_rx_sta & 0x8000) == 0)      /* ����δ��� */
        {
            if(g_usart_rx_sta & 0x4000)         /* ���յ���0x0d */
            {
                if(g_rx_buffer[0] != 0x0a) 
                {
                    g_usart_rx_sta = 0;         /* ���մ���,���¿�ʼ */
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;   /* ��������� */
                }
            }
            else                                /* ��û�յ�0X0D */
            {
                if(g_rx_buffer[0] == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;
                }
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0] ;
                    g_usart_rx_sta++;
                    if(g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;     /* �������ݴ���,���¿�ʼ���� */
                    }
                }
            }
        }
    }
		
		if(huart->Instance == USART3_UX)             /* ����Ǵ���3 */
    {
			if(!(g_usart3_rx_sta& 0x8000))
			{
				g_usart3_rx_buf[g_usart3_rx_sta & 0X3FFF] = g_rx3_buffer[0] ;
				g_usart3_rx_sta++;
				 if(g_usart3_rx_sta > (USART3_REC_LEN - 1))
					{
             g_usart3_rx_sta = 0;     /* �������ݴ���,���¿�ʼ���� */
          }						
				RS232_RX_CNT = g_usart_rx_sta & 0X3FFF;
				if(g_rx3_buffer[0] == 0x51)        /* ���յ���0X51 */
				{
					if(g_usart3_rx_sta!=87)          /* ��;����Ϊ0x51 */
					{
							g_usart3_rx_sta |= 0x4000;   /* �������� */
					}
					else if(g_usart3_rx_buf[0]!=0x57||g_usart3_rx_buf[1]!=0x04)		 
					{
							g_usart3_rx_sta = 0;     /* �������ݴ���,���¿�ʼ���� */
					}
					else 
					{
							g_usart3_rx_sta |= 0x8000;  /*���ݽ�����ȷ*/
							BaseType_t xHigherPriorityTaskWoken;
							xSemaphoreGiveFromISR(usart3Semaphore,&xHigherPriorityTaskWoken);
							portYIELD_FROM_ISR(xHigherPriorityTaskWoken);          /* �����Ҫ�Ļ�����һ�������л�  */
					}
				}
			}
//            if(g_usart3_rx_sta & 0x4000)         /* ���յ���0X51 */
//            {
//								if(g_usart3_rx_buf[0]!=0x57||g_usart3_rx_buf[1]!=0x04)
//								{
//										g_usart3_rx_sta = 0;     /* �������ݴ���,���¿�ʼ���� */
//								}
//								else g_usart3_rx_sta |= 0x8000;  /*���ݽ�����ȷ*/
//            }
//            else                                /* ��û�յ�0X51 ��У��λ81 */
//            {
//									if(g_rx3_buffer[0] == 0x51)
//                {
//                    g_usart3_rx_sta |= 0x4000;
//										g_usart3_rx_buf[g_usart3_rx_sta & 0X3FFF] = g_rx3_buffer[0] ;
//										g_usart3_rx_sta++;
//                }
//                else
//                {
//                    g_usart3_rx_buf[g_usart3_rx_sta & 0X3FFF] = g_rx3_buffer[0] ;
//                    g_usart3_rx_sta++;
//										RS232_RX_CNT = g_usart_rx_sta & 0X3FFF;
//                    if(g_usart3_rx_sta > (USART3_REC_LEN - 1))
//                    {
//                        g_usart3_rx_sta = 0;     /* �������ݴ���,���¿�ʼ���� */
//                    }						
//								}
//                
//            } 
    }
}

/**
 * @brief       ����1�жϷ�����
 * @param       ��
 * @retval      ��
 */
void USART_UX_IRQHandler(void)
{ 
    uint32_t timeout = 0;
    uint32_t maxDelay = 0x1FFFF;
    

    HAL_UART_IRQHandler(&g_uart1_handle);       /* ����HAL���жϴ����ú��� */

    timeout = 0;
    while (HAL_UART_GetState(&g_uart1_handle) != HAL_UART_STATE_READY) /* �ȴ����� */
    {
        timeout++;                              /* ��ʱ���� */
        if(timeout > maxDelay)
        {
            break;
        }
    }
     
    timeout=0;
    
    /* һ�δ������֮�����¿����жϲ�����RxXferCountΪ1 */
    while (HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE) != HAL_OK)
    {
        timeout++;                              /* ��ʱ���� */
        if (timeout > maxDelay)
        {
            break;
        }
    }

}

#endif


 

 #if USART3_EN_RX                                     /* ���ʹ���˽��� */
uint8_t send_232buf[]={0x03,0x04,0xF9};
/* ���ջ���, ���USART3_REC_LEN���ֽ�. */
uint8_t g_usart3_rx_buf[USART3_REC_LEN];


/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x51
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/


uint8_t g_rx3_buffer[RX3BUFFERSIZE];                  /* HAL��ʹ�õĴ��ڽ��ջ��� */

UART_HandleTypeDef g_uart3_handle;                  /* UART��� */

/**
 * @brief       ����3��ʼ������
 * @param       baudrate: ������, �����Լ���Ҫ���ò�����ֵ
 * @note        ע��: ����������ȷ��ʱ��Դ, ���򴮿ڲ����ʾͻ������쳣.
 *              �����USART��ʱ��Դ��sys_stm32_clock_init()�������Ѿ����ù���.
 * @retval      ��
 */
void usart3_init(uint32_t baudrate)
{	
    g_uart3_handle.Instance = USART3_UX;                         /* USART3 */
    g_uart3_handle.Init.BaudRate = baudrate;                    /* ������ */
    g_uart3_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* �ֳ�Ϊ8λ���ݸ�ʽ */
    g_uart3_handle.Init.StopBits = UART_STOPBITS_2;             /* 2��ֹͣλ */
    g_uart3_handle.Init.Parity = UART_PARITY_NONE;              /* ����żУ��λ */
    g_uart3_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* ��Ӳ������ */
    g_uart3_handle.Init.Mode = UART_MODE_TX_RX;                 /* �շ�ģʽ */
    HAL_UART_Init(&g_uart3_handle);                             /* HAL_UART_Init()��ʹ��UART3 */
    
    /* �ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ��������������� */
    HAL_UART_Receive_IT(&g_uart3_handle, (uint8_t *)g_rx3_buffer, RX3BUFFERSIZE);

}

/**
 * @brief       ����3�жϷ�����
 * @param       ��
 * @retval      ��
 */
void USART3_UX_IRQHandler(void)
{ 
    uint32_t timeout = 0;
    uint32_t maxDelay = 0x1FFFF;
    

    HAL_UART_IRQHandler(&g_uart3_handle);       /* ����HAL���жϴ����ú��� */

    timeout = 0;
    while (HAL_UART_GetState(&g_uart3_handle) != HAL_UART_STATE_READY) /* �ȴ����� */
    {
        timeout++;                              /* ��ʱ���� */
        if(timeout > maxDelay)
        {
            break;
        }
    }
     
    timeout=0;
    
    /* һ�δ������֮�����¿����жϲ�����RxXferCountΪ1 */
    while (HAL_UART_Receive_IT(&g_uart3_handle, (uint8_t *)g_rx3_buffer, RX3BUFFERSIZE) != HAL_OK)
    {
        timeout++;                              /* ��ʱ���� */
        if (timeout > maxDelay)
        {
            break;
        }
    }

}

void RS232_Send_Data(uint8_t* buf,uint8_t len)
{
		HAL_UART_Transmit_IT(&g_uart3_handle,buf,len);
		RS232_RX_CNT=0;
}

char *RS232_Receive_Data(void)
{
    uint8_t len;
    char *pbuf = 0;

    if (g_usart3_rx_sta & 0x8000)        /* ���ڽ�����ɣ� */
    {
        len = g_usart3_rx_sta & 0x3fff;  /* �õ��˴ν��յ������ݳ��� */
        g_usart3_rx_buf[len] = '\0';     /* ��ĩβ���������. */
        pbuf = (char*)g_usart3_rx_buf;
        g_usart3_rx_sta = 0;             /* ������һ�ν��� */
    }

    return pbuf;
}


void ECU_data_ask(void){

	RS232_Send_Data(send_232buf,3);
	//printf("������send_232buf[0]:%d [1]:%d [2]:%d",send_232buf[0],send_232buf[1],send_232buf[2]);
//	send_232buf[0] = 0x03;	send_232buf[1] = 0x0B;	send_232buf[2] = 0xF2;
//	RS485_Send_Data(send_232buf,3);	
}

void ECU_data_get(void){
	char* buf;
	buf = RS232_Receive_Data();
	if(buf!=0)
	{
			string_to_u16(&engine_working_state,(uint8_t*)buf,10);  //printf("engine_working_state %d\n",engine_working_state);
			string_to_u16(&engine_speed,(uint8_t*)buf,12);
			string_to_u16(&engine_warm_up,(uint8_t*)buf,18);
			string_to_u16(&engine_idling_set,(uint8_t*)buf,24);
			string_to_u16(&air_inflow,(uint8_t*)buf,28);
			string_to_u16(&throttle_position,(uint8_t*)buf,74);
	}
}

#endif






