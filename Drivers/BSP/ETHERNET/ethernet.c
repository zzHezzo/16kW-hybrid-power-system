/**
 ****************************************************************************************************
 * @file        ethernet.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       ETHERNET ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
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

#include "./BSP/ETHERNET/ethernet.h"
#include "lwip_comm.h"
#include "./SYSTEM/delay/delay.h"
#include "./MALLOC/malloc.h"


ETH_HandleTypeDef g_eth_handler;            /* ��̫����� */
ETH_DMADescTypeDef *g_eth_dma_rx_dscr_tab;  /* ��̫��DMA�������������ݽṹ��ָ�� */
ETH_DMADescTypeDef *g_eth_dma_tx_dscr_tab;  /* ��̫��DMA�������������ݽṹ��ָ�� */
uint8_t *g_eth_rx_buf;                      /* ��̫���ײ���������buffersָ�� */
uint8_t *g_eth_tx_buf;                      /* ��̫���ײ���������buffersָ�� */


/**
 * @brief       ��̫��оƬ��ʼ��
 * @param       ��
 * @retval      0,�ɹ�
 *              1,ʧ��
 */
uint8_t ethernet_init(void)
{
    uint8_t macaddress[6];

    macaddress[0] = g_lwipdev.mac[0];
    macaddress[1] = g_lwipdev.mac[1];
    macaddress[2] = g_lwipdev.mac[2];
    macaddress[3] = g_lwipdev.mac[3];
    macaddress[4] = g_lwipdev.mac[4];
    macaddress[5] = g_lwipdev.mac[5];

    g_eth_handler.Instance = ETH;
    g_eth_handler.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;    /* ʹ����Э��ģʽ */
    g_eth_handler.Init.Speed = ETH_SPEED_100M;                          /* �ٶ�100M,�����������Э��ģʽ�������þ���Ч */
    g_eth_handler.Init.DuplexMode = ETH_MODE_FULLDUPLEX;                /* ȫ˫��ģʽ�������������Э��ģʽ�������þ���Ч */
    g_eth_handler.Init.PhyAddress = ETHERNET_PHY_ADDRESS;               /* ��̫��оƬ�ĵ�ַ */
    g_eth_handler.Init.MACAddr = macaddress;                            /* MAC��ַ */
    g_eth_handler.Init.RxMode = ETH_RXINTERRUPT_MODE;                   /* �жϽ���ģʽ */
    g_eth_handler.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;         /* Ӳ��֡У�� */
    g_eth_handler.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;       /* RMII�ӿ� */

    if (HAL_ETH_Init(&g_eth_handler) == HAL_OK)
    {
        return 0;   /* �ɹ� */
    }
    else
    {
        return 1;  /* ʧ�� */
    }
}

/**
 * @brief       ETH�ײ�������ʱ��ʹ�ܣ���������
 *    @note     �˺����ᱻHAL_ETH_Init()����
 * @param       heth:��̫�����
 * @retval      ��
 */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
    GPIO_InitTypeDef gpio_init_struct;

    ETH_CLK_GPIO_CLK_ENABLE();          /* ����ETH_CLKʱ�� */
    ETH_MDIO_GPIO_CLK_ENABLE();         /* ����ETH_MDIOʱ�� */
    ETH_CRS_GPIO_CLK_ENABLE();          /* ����ETH_CRSʱ�� */
    ETH_MDC_GPIO_CLK_ENABLE();          /* ����ETH_MDCʱ�� */
    ETH_RXD0_GPIO_CLK_ENABLE();         /* ����ETH_RXD0ʱ�� */
    ETH_RXD1_GPIO_CLK_ENABLE();         /* ����ETH_RXD1ʱ�� */
    ETH_TX_EN_GPIO_CLK_ENABLE();        /* ����ETH_TX_ENʱ�� */
    ETH_TXD0_GPIO_CLK_ENABLE();         /* ����ETH_TXD0ʱ�� */
    ETH_TXD1_GPIO_CLK_ENABLE();         /* ����ETH_TXD1ʱ�� */
    ETH_RESET_GPIO_CLK_ENABLE();        /* ����ETH_RESETʱ�� */
    __HAL_RCC_ETH_CLK_ENABLE();         /* ����ETHʱ�� */


    /* ������������ RMII�ӿ�
     * ETH_MDIO -------------------------> PA2
     * ETH_MDC --------------------------> PC1
     * ETH_RMII_REF_CLK------------------> PA1
     * ETH_RMII_CRS_DV ------------------> PA7
     * ETH_RMII_RXD0 --------------------> PC4
     * ETH_RMII_RXD1 --------------------> PC5
     * ETH_RMII_TX_EN -------------------> PG11
     * ETH_RMII_TXD0 --------------------> PG13
     * ETH_RMII_TXD1 --------------------> PG14
     * ETH_RESET-------------------------> PD3
     */

    /* PA1,2,7 */
    gpio_init_struct.Pin = ETH_CLK_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* ���츴�� */
    gpio_init_struct.Pull = GPIO_NOPULL;                    /* ���������� */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;               /* ���� */
    gpio_init_struct.Alternate = GPIO_AF11_ETH;             /* ����ΪETH���� */
    HAL_GPIO_Init(ETH_CLK_GPIO_PORT, &gpio_init_struct);    /* ETH_CLK����ģʽ���� */
    
    gpio_init_struct.Pin = ETH_MDIO_GPIO_PIN;
    HAL_GPIO_Init(ETH_MDIO_GPIO_PORT, &gpio_init_struct);   /* ETH_MDIO����ģʽ���� */
    
    gpio_init_struct.Pin = ETH_CRS_GPIO_PIN;    
    HAL_GPIO_Init(ETH_CRS_GPIO_PORT, &gpio_init_struct);    /* ETH_CRS����ģʽ���� */

    /* PC1 */
    gpio_init_struct.Pin = ETH_MDC_GPIO_PIN;
    HAL_GPIO_Init(ETH_MDC_GPIO_PORT, &gpio_init_struct);    /* ETH_MDC��ʼ�� */

    /* PC4 */
    gpio_init_struct.Pin = ETH_RXD0_GPIO_PIN;
    HAL_GPIO_Init(ETH_RXD0_GPIO_PORT, &gpio_init_struct);   /* ETH_RXD0��ʼ�� */
    
    /* PC5 */
    gpio_init_struct.Pin = ETH_RXD1_GPIO_PIN;
    HAL_GPIO_Init(ETH_RXD1_GPIO_PORT, &gpio_init_struct);   /* ETH_RXD1��ʼ�� */
    
    
    /* PG11,13,14 */
    gpio_init_struct.Pin = ETH_TX_EN_GPIO_PIN; 
    HAL_GPIO_Init(ETH_TX_EN_GPIO_PORT, &gpio_init_struct);  /* ETH_TX_EN��ʼ�� */

    gpio_init_struct.Pin = ETH_TXD0_GPIO_PIN; 
    HAL_GPIO_Init(ETH_TXD0_GPIO_PORT, &gpio_init_struct);   /* ETH_TXD0��ʼ�� */
    
    gpio_init_struct.Pin = ETH_TXD1_GPIO_PIN; 
    HAL_GPIO_Init(ETH_TXD1_GPIO_PORT, &gpio_init_struct);   /* ETH_TXD1��ʼ�� */
    
    
    /* ��λ���� */
    gpio_init_struct.Pin = ETH_RESET_GPIO_PIN;      /* ETH_RESET��ʼ�� */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;    /* ������� */
    gpio_init_struct.Pull = GPIO_NOPULL;            /* �������� */
    gpio_init_struct.Speed = GPIO_SPEED_HIGH;       /* ���� */
    HAL_GPIO_Init(ETH_RESET_GPIO_PORT, &gpio_init_struct);

    ETHERNET_RST(0);     /* Ӳ����λ */
    delay_ms(50);
    ETHERNET_RST(1);     /* ��λ���� */

    HAL_NVIC_SetPriority(ETH_IRQn, 6, 0);           /* �����ж����ȼ�Ӧ�ø�һ�� */
    HAL_NVIC_EnableIRQ(ETH_IRQn);
}

/**
 * @breif       ��ȡ��̫��оƬ�Ĵ���ֵ
 * @param       reg����ȡ�ļĴ�����ַ
 * @retval      ��
 */
uint32_t ethernet_read_phy(uint16_t reg)
{
    uint32_t regval;

    HAL_ETH_ReadPHYRegister(&g_eth_handler, reg, &regval);
    return regval;
}

/**
 * @breif       ����̫��оƬָ����ַд��Ĵ���ֵ
 * @param       reg   : Ҫд��ļĴ���
 * @param       value : Ҫд��ļĴ���
 * @retval      ��
 */
void ethernet_write_phy(uint16_t reg, uint16_t value)
{
    uint32_t temp = value;
    
    HAL_ETH_WritePHYRegister(&g_eth_handler, reg, temp);
}

/**
 * @breif       �������оƬ���ٶ�ģʽ
 * @param       ��
 * @retval      1:��ȡ100M�ɹ�
                0:ʧ��
 */
uint8_t ethernet_chip_get_speed(void)
{
    uint8_t speed;
    #if(PHY_TYPE == LAN8720) 
    speed = ~((ethernet_read_phy(PHY_SR) & PHY_SPEED_STATUS));     /* ��LAN8720��31�żĴ����ж�ȡ�����ٶȺ�˫��ģʽ */
    #elif(PHY_TYPE == SR8201F)
    speed = ((ethernet_read_phy(PHY_SR) & PHY_SPEED_STATUS) >> 13);    /* ��SR8201F��0�żĴ����ж�ȡ�����ٶȺ�˫��ģʽ */
    #elif(PHY_TYPE == YT8512C)
    speed = ((ethernet_read_phy(PHY_SR) & PHY_SPEED_STATUS) >> 14);    /* ��YT8512C��17�żĴ����ж�ȡ�����ٶȺ�˫��ģʽ */
    #elif(PHY_TYPE == RTL8201)
    speed = ((ethernet_read_phy(PHY_SR) & PHY_SPEED_STATUS) >> 1);     /* ��RTL8201��16�żĴ����ж�ȡ�����ٶȺ�˫��ģʽ */
    #endif

    return speed;
}

extern void lwip_pkt_handle(void);                  /* ��lwip_comm.c���涨�� */

/**
 * @breif       �жϷ�����
 * @param       ��
 * @retval      ��
 */
void ETH_IRQHandler(void)
{
    if (ethernet_get_eth_rx_size(g_eth_handler.RxDesc))
    {
        lwip_pkt_handle();      /* ������̫�����ݣ����������ύ��LWIP */
    }

    __HAL_ETH_DMA_CLEAR_IT(&g_eth_handler, ETH_DMA_IT_NIS);   /* ���DMA�жϱ�־λ */
    __HAL_ETH_DMA_CLEAR_IT(&g_eth_handler, ETH_DMA_IT_R);     /* ���DMA�����жϱ�־λ */
}

/**
 * @breif       ��ȡ���յ���֡����
 * @param       dma_rx_desc : ����DMA������
 * @retval      frameLength : ���յ���֡����
 */
uint32_t  ethernet_get_eth_rx_size(ETH_DMADescTypeDef *dma_rx_desc)
{
    uint32_t frameLength = 0;

    if (((dma_rx_desc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET) &&
        ((dma_rx_desc->Status & ETH_DMARXDESC_ES)  == (uint32_t)RESET) &&
        ((dma_rx_desc->Status & ETH_DMARXDESC_LS)  != (uint32_t)RESET))
    {
        frameLength = ((dma_rx_desc->Status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAME_LENGTHSHIFT);
    }

    return frameLength;
}

/**
 * @breif       ΪETH�ײ����������ڴ�
 * @param       ��
 * @retval      0,����
 *              1,ʧ��
 */
uint8_t ethernet_mem_malloc(void)
{
    if ((g_eth_dma_rx_dscr_tab || g_eth_dma_tx_dscr_tab || g_eth_rx_buf || g_eth_tx_buf) == NULL)
    {
        g_eth_dma_rx_dscr_tab = mymalloc(SRAMIN, ETH_RXBUFNB * sizeof(ETH_DMADescTypeDef));         /* �����ڴ� */
        g_eth_dma_tx_dscr_tab = mymalloc(SRAMIN, ETH_TXBUFNB * sizeof(ETH_DMADescTypeDef));         /* �����ڴ� */
        g_eth_rx_buf = mymalloc(SRAMIN, ETH_RX_BUF_SIZE * ETH_RXBUFNB);                             /* �����ڴ� */
        g_eth_tx_buf = mymalloc(SRAMIN, ETH_TX_BUF_SIZE * ETH_TXBUFNB);                             /* �����ڴ� */
        
        if (!(uint32_t)&g_eth_dma_rx_dscr_tab || !(uint32_t)&g_eth_dma_tx_dscr_tab || !(uint32_t)&g_eth_rx_buf || !(uint32_t)&g_eth_tx_buf)
        {
            ethernet_mem_free();
            return 1;                                                                               /* ����ʧ�� */
        }
    }

    return 0;       /* ����ɹ� */
}

/**
 * @breif       �ͷ�ETH �ײ�����������ڴ�
 * @param       ��
 * @retval      ��
 */
void ethernet_mem_free(void)
{
    myfree(SRAMIN, g_eth_dma_rx_dscr_tab);  /* �ͷ��ڴ� */
    myfree(SRAMIN, g_eth_dma_tx_dscr_tab);  /* �ͷ��ڴ� */
    myfree(SRAMIN, g_eth_rx_buf);           /* �ͷ��ڴ� */
    myfree(SRAMIN, g_eth_tx_buf);           /* �ͷ��ڴ� */
}
