#ifndef __RS485_H
#define __RS485_H			 							  

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"

/*******************************************************************************************************/
/* ���� �� ���� ���� 
 * Ĭ�������USART1��.
 * ע��: ͨ���޸���12���궨��,����֧��USART1~UART7����һ������.
 * �޸�Ϊ֧��com3 ����3��rs232ͨѶ USART3
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
#define USART3_UX_IRQHandler             USART3_IRQHandler//USART1_IRQHandler
#define USART3_UX_CLK_ENABLE()           do{ __HAL_RCC_USART3_CLK_ENABLE(); }while(0)  /* USART3 ʱ��ʹ�� */

/*******************************************************************************************************/

#define USART3_REC_LEN   110                    /* �����������ֽ��� 110 */
#define USART3_EN_RX     1                       /* ʹ�ܣ�1��/��ֹ��0������3���� */
#define RX3BUFFERSIZE    1                       /* �����С */

extern UART_HandleTypeDef g_uart3_handle;       /* UART��� */

extern uint8_t  g_usart3_rx_buf[USART3_REC_LEN];  /* ���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� */
extern uint16_t g_usart3_rx_sta;                 /* ����״̬��� */
extern uint8_t g_rx3_buffer[RX3BUFFERSIZE];       /* HAL��USART����Buffer */


void usart3_init(uint32_t baudrate);             /* ���ڳ�ʼ������ */

#endif


	  	
//extern u8 RS485_RX_BUF[100]; 		//���ջ���,���100���ֽ�
//extern u8 RS485_RX_CNT;   			//���յ������ݳ���

////ģʽ����
//#define RS485_TX_EN		PGout(8)	//485ģʽ����.0,����;1,����.
////����봮���жϽ��գ�����EN_USART2_RXΪ1����������Ϊ0
//#define EN_USART2_RX 	1			//0,������;1,����.

//														 
//void RS485_Init(u32 bound);
//void RS485_Send_Data(u8 *buf,u8 len);
//void RS485_Receive_Data(u8 *buf,u8 *len);		 
//#endif	   
















