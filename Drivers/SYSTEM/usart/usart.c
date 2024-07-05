/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       串口初始化代码(一般是串口1)，支持printf
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
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
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/***********************************************END*******************************************/



#if USART_EN_RX                                     /* 如果使能了接收 */

/* 接收缓冲, 最大USART_REC_LEN个字节. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x0d
 *  bit13~0，    接收到的有效字节数目
*/
uint16_t g_usart_rx_sta = 0;

uint16_t g_usart3_rx_sta = 0;
uint8_t RS232_RX_CNT=0; 


	
uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* HAL库使用的串口接收缓冲 */

UART_HandleTypeDef g_uart1_handle;                  /* UART句柄 */


/**
 * @brief       串口X初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart_init(uint32_t baudrate)
{
    g_uart1_handle.Instance = USART_UX;                         /* USART1 */
    g_uart1_handle.Init.BaudRate = baudrate;                    /* 波特率 */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* 字长为8位数据格式 */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;             /* 一个停止位 */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;              /* 无奇偶校验位 */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* 无硬件流控 */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;                 /* 收发模式 */
    HAL_UART_Init(&g_uart1_handle);                             /* HAL_UART_Init()会使能UART1 */
    
    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    if(huart->Instance == USART_UX)                             /* 如果是串口1，进行串口1 MSP初始化 */
    {
        USART_UX_CLK_ENABLE();                                  /* USART1 时钟使能 */
        USART_TX_GPIO_CLK_ENABLE();                             /* 发送引脚时钟使能 */
        USART_RX_GPIO_CLK_ENABLE();                             /* 接收引脚时钟使能 */

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;               /* TX引脚 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;          /* 复用为USART1 */
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);   /* 初始化发送引脚 */

        gpio_init_struct.Pin = USART_RX_GPIO_PIN;               /* RX引脚 */
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;          /* 复用为USART1 */
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);   /* 初始化接收引脚 */

#if USART_EN_RX
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                      /* 使能USART1中断通道 */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 0);              /* 抢占优先级3，子优先级0 */
#endif
			
			
		if(huart->Instance == USART3_UX)                             /* 如果是串口3，进行串口3 MSP初始化 */
    {
        USART3_UX_CLK_ENABLE();                                  /* USART3 时钟使能 */
        USART3_TX_GPIO_CLK_ENABLE();                             /* 发送引脚时钟使能 */
        USART3_RX_GPIO_CLK_ENABLE();                             /* 接收引脚时钟使能 */

        gpio_init_struct.Pin = USART3_TX_GPIO_PIN;               /* TX引脚 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
        gpio_init_struct.Alternate = USART3_TX_GPIO_AF;          /* 复用为USART3 */
        HAL_GPIO_Init(USART3_TX_GPIO_PORT, &gpio_init_struct);   /* 初始化发送引脚 */

        gpio_init_struct.Pin = USART3_RX_GPIO_PIN;               /* RX引脚 */
        gpio_init_struct.Alternate = USART3_RX_GPIO_AF;          /* 复用为USART3 */
        HAL_GPIO_Init(USART3_RX_GPIO_PORT, &gpio_init_struct);   /* 初始化接收引脚 */

#if USART3_EN_RX
        HAL_NVIC_EnableIRQ(USART3_UX_IRQn);                      /* 使能USART3中断通道 */
        HAL_NVIC_SetPriority(USART3_UX_IRQn, 7, 0);              /* 抢占优先级7，子优先级0 */
#endif
    }
    }
}

/**
 * @brief       Rx传输回调函数
 * @param       huart: UART句柄类型指针
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART_UX)             /* 如果是串口1 */
    {
        if((g_usart_rx_sta & 0x8000) == 0)      /* 接收未完成 */
        {
            if(g_usart_rx_sta & 0x4000)         /* 接收到了0x0d */
            {
                if(g_rx_buffer[0] != 0x0a) 
                {
                    g_usart_rx_sta = 0;         /* 接收错误,重新开始 */
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;   /* 接收完成了 */
                }
            }
            else                                /* 还没收到0X0D */
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
                        g_usart_rx_sta = 0;     /* 接收数据错误,重新开始接收 */
                    }
                }
            }
        }
    }
		
		if(huart->Instance == USART3_UX)             /* 如果是串口3 */
    {
			if(!(g_usart3_rx_sta& 0x8000))
			{
				g_usart3_rx_buf[g_usart3_rx_sta & 0X3FFF] = g_rx3_buffer[0] ;
				g_usart3_rx_sta++;
				 if(g_usart3_rx_sta > (USART3_REC_LEN - 1))
					{
             g_usart3_rx_sta = 0;     /* 接收数据错误,重新开始接收 */
          }						
				RS232_RX_CNT = g_usart_rx_sta & 0X3FFF;
				if(g_rx3_buffer[0] == 0x51)        /* 接收到了0X51 */
				{
					if(g_usart3_rx_sta!=87)          /* 中途有数为0x51 */
					{
							g_usart3_rx_sta |= 0x4000;   /* 继续接收 */
					}
					else if(g_usart3_rx_buf[0]!=0x57||g_usart3_rx_buf[1]!=0x04)		 
					{
							g_usart3_rx_sta = 0;     /* 接收数据错误,重新开始接收 */
					}
					else 
					{
							g_usart3_rx_sta |= 0x8000;  /*数据接收正确*/
							BaseType_t xHigherPriorityTaskWoken;
							xSemaphoreGiveFromISR(usart3Semaphore,&xHigherPriorityTaskWoken);
							portYIELD_FROM_ISR(xHigherPriorityTaskWoken);          /* 如果需要的话进行一次任务切换  */
					}
				}
			}
//            if(g_usart3_rx_sta & 0x4000)         /* 接收到了0X51 */
//            {
//								if(g_usart3_rx_buf[0]!=0x57||g_usart3_rx_buf[1]!=0x04)
//								{
//										g_usart3_rx_sta = 0;     /* 接收数据错误,重新开始接收 */
//								}
//								else g_usart3_rx_sta |= 0x8000;  /*数据接收正确*/
//            }
//            else                                /* 还没收到0X51 即校验位81 */
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
//                        g_usart3_rx_sta = 0;     /* 接收数据错误,重新开始接收 */
//                    }						
//								}
//                
//            } 
    }
}

/**
 * @brief       串口1中断服务函数
 * @param       无
 * @retval      无
 */
void USART_UX_IRQHandler(void)
{ 
    uint32_t timeout = 0;
    uint32_t maxDelay = 0x1FFFF;
    

    HAL_UART_IRQHandler(&g_uart1_handle);       /* 调用HAL库中断处理公用函数 */

    timeout = 0;
    while (HAL_UART_GetState(&g_uart1_handle) != HAL_UART_STATE_READY) /* 等待就绪 */
    {
        timeout++;                              /* 超时处理 */
        if(timeout > maxDelay)
        {
            break;
        }
    }
     
    timeout=0;
    
    /* 一次处理完成之后，重新开启中断并设置RxXferCount为1 */
    while (HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE) != HAL_OK)
    {
        timeout++;                              /* 超时处理 */
        if (timeout > maxDelay)
        {
            break;
        }
    }

}

#endif


 

 #if USART3_EN_RX                                     /* 如果使能了接收 */
uint8_t send_232buf[]={0x03,0x04,0xF9};
/* 接收缓冲, 最大USART3_REC_LEN个字节. */
uint8_t g_usart3_rx_buf[USART3_REC_LEN];


/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x51
 *  bit13~0，    接收到的有效字节数目
*/


uint8_t g_rx3_buffer[RX3BUFFERSIZE];                  /* HAL库使用的串口接收缓冲 */

UART_HandleTypeDef g_uart3_handle;                  /* UART句柄 */

/**
 * @brief       串口3初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart3_init(uint32_t baudrate)
{	
    g_uart3_handle.Instance = USART3_UX;                         /* USART3 */
    g_uart3_handle.Init.BaudRate = baudrate;                    /* 波特率 */
    g_uart3_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* 字长为8位数据格式 */
    g_uart3_handle.Init.StopBits = UART_STOPBITS_2;             /* 2个停止位 */
    g_uart3_handle.Init.Parity = UART_PARITY_NONE;              /* 无奇偶校验位 */
    g_uart3_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* 无硬件流控 */
    g_uart3_handle.Init.Mode = UART_MODE_TX_RX;                 /* 收发模式 */
    HAL_UART_Init(&g_uart3_handle);                             /* HAL_UART_Init()会使能UART3 */
    
    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    HAL_UART_Receive_IT(&g_uart3_handle, (uint8_t *)g_rx3_buffer, RX3BUFFERSIZE);

}

/**
 * @brief       串口3中断服务函数
 * @param       无
 * @retval      无
 */
void USART3_UX_IRQHandler(void)
{ 
    uint32_t timeout = 0;
    uint32_t maxDelay = 0x1FFFF;
    

    HAL_UART_IRQHandler(&g_uart3_handle);       /* 调用HAL库中断处理公用函数 */

    timeout = 0;
    while (HAL_UART_GetState(&g_uart3_handle) != HAL_UART_STATE_READY) /* 等待就绪 */
    {
        timeout++;                              /* 超时处理 */
        if(timeout > maxDelay)
        {
            break;
        }
    }
     
    timeout=0;
    
    /* 一次处理完成之后，重新开启中断并设置RxXferCount为1 */
    while (HAL_UART_Receive_IT(&g_uart3_handle, (uint8_t *)g_rx3_buffer, RX3BUFFERSIZE) != HAL_OK)
    {
        timeout++;                              /* 超时处理 */
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

    if (g_usart3_rx_sta & 0x8000)        /* 串口接收完成？ */
    {
        len = g_usart3_rx_sta & 0x3fff;  /* 得到此次接收到的数据长度 */
        g_usart3_rx_buf[len] = '\0';     /* 在末尾加入结束符. */
        pbuf = (char*)g_usart3_rx_buf;
        g_usart3_rx_sta = 0;             /* 开启下一次接收 */
    }

    return pbuf;
}


void ECU_data_ask(void){

	RS232_Send_Data(send_232buf,3);
	//printf("发送了send_232buf[0]:%d [1]:%d [2]:%d",send_232buf[0],send_232buf[1],send_232buf[2]);
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






