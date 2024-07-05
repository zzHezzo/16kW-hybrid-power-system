#ifndef _USART_H
#define _USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"

/*******************************************************************************************************/
/* 引脚 和 串口 定义 
 * 默认是针对USART1的.
 * 注意: 通过修改这12个宏定义,可以支持USART1~UART7任意一个串口.
 */

#define USART_TX_GPIO_PORT              GPIOA
#define USART_TX_GPIO_PIN               GPIO_PIN_9
#define USART_TX_GPIO_AF                GPIO_AF7_USART1
#define USART_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* 发送引脚时钟使能 */

#define USART_RX_GPIO_PORT              GPIOA
#define USART_RX_GPIO_PIN               GPIO_PIN_10
#define USART_RX_GPIO_AF                GPIO_AF7_USART1
#define USART_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* 接收引脚时钟使能 */

#define USART_UX                        USART1
#define USART_UX_IRQn                   USART1_IRQn
#define USART_UX_IRQHandler             USART1_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ __HAL_RCC_USART1_CLK_ENABLE(); }while(0)  /* USART1 时钟使能 */

/*******************************************************************************************************/

#define USART_REC_LEN   200                     /* 定义最大接收字节数 200 */
#define USART_EN_RX     1                       /* 使能（1）/禁止（0）串口1接收 */
#define RXBUFFERSIZE    1                       /* 缓存大小 */

/*******************************************************************************************************/

/*
 * USART3引脚串口定义
 */
#define USART3_TX_GPIO_PORT              GPIOB
#define USART3_TX_GPIO_PIN               GPIO_PIN_10
#define USART3_TX_GPIO_AF                GPIO_AF7_USART3
#define USART3_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* 发送引脚时钟使能 */

#define USART3_RX_GPIO_PORT              GPIOB
#define USART3_RX_GPIO_PIN               GPIO_PIN_11
#define USART3_RX_GPIO_AF                GPIO_AF7_USART3
#define USART3_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* 接收引脚时钟使能 */

#define USART3_UX                        USART3
#define USART3_UX_IRQn                   USART3_IRQn
#define USART3_UX_IRQHandler             USART3_IRQHandler
#define USART3_UX_CLK_ENABLE()           do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)  /* USART3 时钟使能 */

/*******************************************************************************************************/

#define USART3_REC_LEN   100                    /* 定义最大接收字节数 100 */
#define USART3_EN_RX     1                       /* 使能（1）/禁止（0）串口3接收 */
#define RX3BUFFERSIZE    1                       /* 缓存大小 */

/*******************************************************************************************************/

extern UART_HandleTypeDef g_uart3_handle;       /* UART3句柄 */

extern uint8_t  g_usart3_rx_buf[USART3_REC_LEN];  /* 接收缓冲,最大USART3_REC_LEN个字节.末字节为校验符 */
extern uint16_t g_usart3_rx_sta;                 /* 接收状态标记 */
extern uint8_t g_rx3_buffer[RX3BUFFERSIZE];       /* HAL库USART3接收Buffer */


void usart3_init(uint32_t baudrate);             /* 串口初始化函数 */

void RS232_Send_Data(uint8_t* buf,uint8_t len);

void ECU_data_ask(void);
void ECU_data_get(void);
/*******************************************************************************************************/


extern UART_HandleTypeDef g_uart1_handle;       /* UART句柄 */

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 */
extern uint16_t g_usart_rx_sta;                 /* 接收状态标记 */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL库USART接收Buffer */


void usart_init(uint32_t baudrate);             /* 串口初始化函数 */

#endif







