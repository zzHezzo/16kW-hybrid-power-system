#include "./SYSTEM/sys/sys.h"    
#include "rs485.h"	 
#include "./SYSTEM/delay/delay.h"
#if USART3_EN_RX                                     /* 如果使能了接收 */

/* 接收缓冲, 最大USART_REC_LEN个字节. */
uint8_t g_usart_rx_buf[USART3_REC_LEN];

/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x0d
 *  bit13~0，    接收到的有效字节数目
*/
uint16_t g_usart_rx_sta = 0;

uint8_t g_rx_buffer[RX3BUFFERSIZE];                  /* HAL库使用的串口接收缓冲 */

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
    HAL_UART_Receive_IT(&g_uart3_handle, (uint8_t *)g_rx_buffer, RX3BUFFERSIZE);
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART3_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
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

/**
 * @brief       Rx传输回调函数
 * @param       huart: UART句柄类型指针
 * @retval      无
 */
void HAL_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3_UX)             /* 如果是串口3 */
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
                    if(g_usart_rx_sta > (USART3_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;     /* 接收数据错误,重新开始接收 */
                    }
                }
            }
        }
    }
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
    while (HAL_UART_Receive_IT(&g_uart3_handle, (uint8_t *)g_rx_buffer, RX3BUFFERSIZE) != HAL_OK)
    {
        timeout++;                              /* 超时处理 */
        if (timeout > maxDelay)
        {
            break;
        }
    }

}

#endif

#if USART3_EN_RX   		//如果使能了接收   	  
//接收缓存区 	
uint8_t RS485_RX_BUF[100];  	//接收缓冲,最大100个字节.
//接收到的数据长度
uint8_t RS485_RX_CNT=0;   
void USART2_IRQHandler(void)
{
	uint8_t res;	    
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
	{	 	
	  res =USART_ReceiveData(USART2);//;读取接收到的数据USART2->DR
		if(RS485_RX_CNT<100)
		{
			RS485_RX_BUF[RS485_RX_CNT]=res;		//记录接收到的值
			RS485_RX_CNT++;						//接收数据增加1 
		} 
	}  											 
} 
//#endif										 
////初始化IO 串口2
////bound:波特率	  
//void RS485_Init(uint32_t bound)
//{  	 
//	
//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
//	
//  //串口2引脚复用映射
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
//	
//	//USART2    
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
//	
//	//PG8推挽输出，485模式控制  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //GPIOG8
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化PG8
//	

//   //USART2 初始化设置
//	USART_InitStructure.USART_BaudRate = bound;//波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//  USART_Init(USART2, &USART_InitStructure); //初始化串口2
//	
//  USART_Cmd(USART2, ENABLE);  //使能串口 2
//	
//	USART_ClearFlag(USART2, USART_FLAG_TC);
//	
//#if EN_USART2_RX	
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启接受中断

//	//Usart2 NVIC 配置
//  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

//#endif	
//	
//	RS485_TX_EN=0;				//默认为接收模式	
//}

////RS485发送len个字节.
////buf:发送区首地址
////len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过100个字节)
//void RS485_Send_Data(uint8_t *buf,uint8_t len)
//{
//	uint8_t t;
//	RS485_TX_EN=1;			//设置为发送模式
//  	for(t=0;t<len;t++)		//循环发送数据
//	{
//	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //等待发送结束		
//    USART_SendData(USART2,buf[t]); //发送数据
//	}	 
//	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //等待发送结束		
//	RS485_RX_CNT=0;	  
//	RS485_TX_EN=0;				//设置为接收模式	
//}
////RS485查询接收到的数据
////buf:接收缓存首地址
////len:读到的数据长度
//void RS485_Receive_Data(uint8_t *buf,uint8_t *len)
//{
//	uint8_t rxlen=RS485_RX_CNT;
//	uint8_t i=0;
//	*len=0;				//默认为0
//	delay_ms(10);		//等待10ms,连续超过10ms没有接收到一个数据,则认为接收结束
//	if(rxlen==RS485_RX_CNT&&rxlen)//接收到了数据,且接收完成了
//	{
//		for(i=0;i<rxlen;i++)
//		{
//			buf[i]=RS485_RX_BUF[i];	
//		}		
//		*len=RS485_RX_CNT;	//记录本次数据长度
//		RS485_RX_CNT=0;		//清零
//	}
//}





