#ifndef _USART_H
#define _USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"

/*******************************************************************************************************/
/* ���� �� ���� ���� 
 * Ĭ�������USART1��.
 * ע��: ͨ���޸���12���궨��,����֧��USART1~UART7����һ������.
 */

#define USART_TX_GPIO_PORT              GPIOA
#define USART_TX_GPIO_PIN               GPIO_PIN_9
#define USART_TX_GPIO_AF                GPIO_AF7_USART1
#define USART_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART_RX_GPIO_PORT              GPIOA
#define USART_RX_GPIO_PIN               GPIO_PIN_10
#define USART_RX_GPIO_AF                GPIO_AF7_USART1
#define USART_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART_UX                        USART1
#define USART_UX_IRQn                   USART1_IRQn
#define USART_UX_IRQHandler             USART1_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ __HAL_RCC_USART1_CLK_ENABLE(); }while(0)  /* USART1 ʱ��ʹ�� */

/*******************************************************************************************************/

#define USART_REC_LEN   200                     /* �����������ֽ��� 200 */
#define USART_EN_RX     1                       /* ʹ�ܣ�1��/��ֹ��0������1���� */
#define RXBUFFERSIZE    1                       /* �����С */

/*******************************************************************************************************/

/*
 * USART3���Ŵ��ڶ���
 */
#define USART3_TX_GPIO_PORT              GPIOB
#define USART3_TX_GPIO_PIN               GPIO_PIN_10
#define USART3_TX_GPIO_AF                GPIO_AF7_USART3
#define USART3_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART3_RX_GPIO_PORT              GPIOB
#define USART3_RX_GPIO_PIN               GPIO_PIN_11
#define USART3_RX_GPIO_AF                GPIO_AF7_USART3
#define USART3_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* ��������ʱ��ʹ�� */

#define USART3_UX                        USART3
#define USART3_UX_IRQn                   USART3_IRQn
#define USART3_UX_IRQHandler             USART3_IRQHandler
#define USART3_UX_CLK_ENABLE()           do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)  /* USART3 ʱ��ʹ�� */

/*******************************************************************************************************/

#define USART3_REC_LEN   100                    /* �����������ֽ��� 100 */
#define USART3_EN_RX     1                       /* ʹ�ܣ�1��/��ֹ��0������3���� */
#define RX3BUFFERSIZE    1                       /* �����С */

/*******************************************************************************************************/

extern UART_HandleTypeDef g_uart3_handle;       /* UART3��� */

extern uint8_t  g_usart3_rx_buf[USART3_REC_LEN];  /* ���ջ���,���USART3_REC_LEN���ֽ�.ĩ�ֽ�ΪУ��� */
extern uint16_t g_usart3_rx_sta;                 /* ����״̬��� */
extern uint8_t g_rx3_buffer[RX3BUFFERSIZE];       /* HAL��USART3����Buffer */


void usart3_init(uint32_t baudrate);             /* ���ڳ�ʼ������ */

void RS232_Send_Data(uint8_t* buf,uint8_t len);

void ECU_data_ask(void);
void ECU_data_get(void);
/*******************************************************************************************************/


extern UART_HandleTypeDef g_uart1_handle;       /* UART��� */

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* ���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
extern uint16_t g_usart_rx_sta;                 /* ����״̬��� */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL��USART����Buffer */


void usart_init(uint32_t baudrate);             /* ���ڳ�ʼ������ */

#endif







