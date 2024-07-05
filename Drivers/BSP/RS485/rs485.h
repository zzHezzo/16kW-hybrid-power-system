#ifndef __RS485_H
#define __RS485_H			 							  

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"

/*******************************************************************************************************/
/* 引脚 和 串口 定义 
 * 默认是针对USART1的.
 * 注意: 通过修改这12个宏定义,可以支持USART1~UART7任意一个串口.
 * 修改为支持com3 串口3的rs232通讯 USART3
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
#define USART3_UX_IRQHandler             USART3_IRQHandler//USART1_IRQHandler
#define USART3_UX_CLK_ENABLE()           do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)  /* USART3 时钟使能 */

/*******************************************************************************************************/

#define USART3_REC_LEN   110                    /* 定义最大接收字节数 110 */
#define USART3_EN_RX     1                       /* 使能（1）/禁止（0）串口3接收 */
#define RX3BUFFERSIZE    1                       /* 缓存大小 */

extern UART_HandleTypeDef g_uart3_handle;       /* UART句柄 */

extern uint8_t  g_usart3_rx_buf[USART3_REC_LEN];  /* 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 */
extern uint16_t g_usart3_rx_sta;                 /* 接收状态标记 */
extern uint8_t g_rx3_buffer[RX3BUFFERSIZE];       /* HAL库USART接收Buffer */


void usart3_init(uint32_t baudrate);             /* 串口初始化函数 */

#endif


	  	
//extern u8 RS485_RX_BUF[100]; 		//接收缓冲,最大100个字节
//extern u8 RS485_RX_CNT;   			//接收到的数据长度

////模式控制
//#define RS485_TX_EN		PGout(8)	//485模式控制.0,接收;1,发送.
////如果想串口中断接收，设置EN_USART2_RX为1，否则设置为0
//#define EN_USART2_RX 	1			//0,不接收;1,接收.

//														 
//void RS485_Init(u32 bound);
//void RS485_Send_Data(u8 *buf,u8 len);
//void RS485_Receive_Data(u8 *buf,u8 *len);		 
//#endif	   
















