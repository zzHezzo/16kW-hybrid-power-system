#include "./BSP/SPI/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"

SPI_HandleTypeDef g_spi1_handler; /* SPI1��� */
SPI_HandleTypeDef g_spi2_handler; /* SPI2��� */

/**
 * @brief       MAX6675��ʼ������
 *   @note      ����ģʽ,8λ����,��ֹӲ��Ƭѡ
 * @param       ��
 * @retval      ��
 */
void spi1_MAX6675_init(void)
{
    SPI1_SPI_CLK_ENABLE(); /* SPI1ʱ��ʹ�� */

    g_spi1_handler.Instance = SPI1_SPI;                                /* SPI1 */
    g_spi1_handler.Init.Mode = SPI_MODE_MASTER;                        /* ����SPI����ģʽ������Ϊ��ģʽ */
    g_spi1_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* ����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ */
    g_spi1_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* ����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ */
    g_spi1_handler.Init.CLKPolarity = SPI_POLARITY_LOW;                /* ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ */
    g_spi1_handler.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* ����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ����� */
    g_spi1_handler.Init.NSS = SPI_NSS_SOFT;                            /* NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ���� */
    g_spi1_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  /* ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16 MAX6675������4.3MHz APB2 84MHz*/
    g_spi1_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ */
    g_spi1_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* �ر�TIģʽ */
    g_spi1_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* �ر�Ӳ��CRCУ�� */
    g_spi1_handler.Init.CRCPolynomial = 7;                             /* CRCֵ����Ķ���ʽ */
    HAL_SPI_Init(&g_spi1_handler);                                     /* ��ʼ�� */

    __HAL_SPI_ENABLE(&g_spi1_handler); /* ʹ��SPI1 */

    //spi_read_write_byte(0XFF,&g_spi1_handler); /* ��������, ʵ���Ͼ��ǲ���8��ʱ������, �ﵽ���DR������, �Ǳ��� */
}

void spi2_MAX6675_init(void)
{
    SPI2_SPI_CLK_ENABLE(); /* SPI2ʱ��ʹ�� */

    g_spi2_handler.Instance = SPI2_SPI;                                /* SPI2 */
    g_spi2_handler.Init.Mode = SPI_MODE_MASTER;                        /* ����SPI����ģʽ������Ϊ��ģʽ */
    g_spi2_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* ����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ */
    g_spi2_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* ����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ */
    g_spi2_handler.Init.CLKPolarity = SPI_POLARITY_LOW;                /* ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ */
    g_spi2_handler.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* ����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ����� */
    g_spi2_handler.Init.NSS = SPI_NSS_SOFT;                            /* NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ���� */
    g_spi2_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; 	 /* ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8 spi2����APB1 42MHz */
    g_spi2_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ */
    g_spi2_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* �ر�TIģʽ */
    g_spi2_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* �ر�Ӳ��CRCУ�� */
    g_spi2_handler.Init.CRCPolynomial = 7;                             /* CRCֵ����Ķ���ʽ */
    HAL_SPI_Init(&g_spi2_handler);                                     /* ��ʼ�� */

    __HAL_SPI_ENABLE(&g_spi2_handler); /* ʹ��SPI2 */

    //spi_read_write_byte(0XFF,&g_spi2_handler); /* ��������, ʵ���Ͼ��ǲ���8��ʱ������, �ﵽ���DR������, �Ǳ��� */
}


/**
 * @brief       SPI��ʼ������
 *   @note      ����ģʽ,8λ����,��ֹӲ��Ƭѡ
 * @param       ��
 * @retval      ��
 */
//void spi1_init(void)
//{
//    SPI1_SPI_CLK_ENABLE(); /* SPI1ʱ��ʹ�� */

//    g_spi1_handler.Instance = SPI1_SPI;                                /* SPI1 */
//    g_spi1_handler.Init.Mode = SPI_MODE_MASTER;                        /* ����SPI����ģʽ������Ϊ��ģʽ */
//    g_spi1_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* ����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ */
//    g_spi1_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* ����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ */
//    g_spi1_handler.Init.CLKPolarity = SPI_POLARITY_LOW;               /* ����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ */
//    g_spi1_handler.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* ����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ����� */
//    g_spi1_handler.Init.NSS = SPI_NSS_SOFT;                            /* NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ���� */
//    g_spi1_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256 */
//    g_spi1_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ */
//    g_spi1_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* �ر�TIģʽ */
//    g_spi1_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* �ر�Ӳ��CRCУ�� */
//    g_spi1_handler.Init.CRCPolynomial = 7;                             /* CRCֵ����Ķ���ʽ */
//    HAL_SPI_Init(&g_spi1_handler);                                     /* ��ʼ�� */

//    __HAL_SPI_ENABLE(&g_spi1_handler); /* ʹ��SPI1 */

//    spi1_read_write_byte(0XFF); /* ��������, ʵ���Ͼ��ǲ���8��ʱ������, �ﵽ���DR������, �Ǳ��� */
//}

/**
 * @brief       SPI1�ײ�������ʱ��ʹ�ܣ���������
 *   @note      �˺����ᱻHAL_SPI_Init()����
 * @param       hspi:SPI���
 * @retval      ��
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_init_struct;
    if (hspi->Instance == SPI1_SPI)
    {
				SPI1_CS_GPIO_CLK_ENABLE();	/* SPI1_CS��ʱ��ʹ�� */
        SPI1_SCK_GPIO_CLK_ENABLE();  /* SPI1_SCK��ʱ��ʹ�� */
        SPI1_MISO_GPIO_CLK_ENABLE(); /* SPI1_MISO��ʱ��ʹ�� */
        SPI1_MOSI_GPIO_CLK_ENABLE(); /* SPI1_MOSI��ʱ��ʹ�� */
			
        /* SCK����ģʽ����(�������) */
        gpio_init_struct.Pin = SPI1_SCK_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(SPI1_SCK_GPIO_PORT, &gpio_init_struct);

			  /* MOSI����ģʽ����(�������) */
        gpio_init_struct.Pin = SPI1_MOSI_GPIO_PIN;
        HAL_GPIO_Init(SPI1_MOSI_GPIO_PORT, &gpio_init_struct);
        
				/* MISO����ģʽ����(�������) */
				//gpio_init_struct.Mode = GPIO_MODE_INPUT;
        gpio_init_struct.Pin = SPI1_MISO_GPIO_PIN;
        HAL_GPIO_Init(SPI1_MISO_GPIO_PORT, &gpio_init_struct);	

				/*Ƭѡ��ʼ�� PA4*/
				gpio_init_struct.Pin = SPI1_CS_GPIO_PIN;
				gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
				//gpio_init_struct.Pull = GPIO_PULLUP;
				gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
				HAL_GPIO_Init(SPI1_CS_GPIO_PORT, &gpio_init_struct);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT,SPI1_CS_GPIO_PIN,GPIO_PIN_SET);//�Ȱ�Ƭѡ����
    }
		
    if (hspi->Instance == SPI2_SPI)
    {
				SPI2_CS_GPIO_CLK_ENABLE();	/* SPI2_CS��ʱ��ʹ�� */
        SPI2_SCK_GPIO_CLK_ENABLE();  /* SPI2_SCK��ʱ��ʹ�� */
        SPI2_MISO_GPIO_CLK_ENABLE(); /* SPI2_MISO��ʱ��ʹ�� */
        SPI2_MOSI_GPIO_CLK_ENABLE(); /* SPI2_MOSI��ʱ��ʹ�� */

        /* SCK����ģʽ����(�������) */
        gpio_init_struct.Pin = SPI2_SCK_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(SPI2_SCK_GPIO_PORT, &gpio_init_struct);

			  /* MOSI����ģʽ����(�������) */
        gpio_init_struct.Pin = SPI2_MOSI_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MOSI_GPIO_PORT, &gpio_init_struct);
        
				/* MISO����ģʽ����(�������) */
        gpio_init_struct.Pin = SPI2_MISO_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MISO_GPIO_PORT, &gpio_init_struct);
			
				/*Ƭѡ��ʼ�� PB12*/
				gpio_init_struct.Pin = SPI2_CS_GPIO_PIN;
				gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
				HAL_GPIO_Init(SPI2_CS_GPIO_PORT, &gpio_init_struct);
				HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT,SPI2_CS_GPIO_PIN,GPIO_PIN_SET);
    }
}

uint16_t MAX6675_ReadTemprature(SPI_HandleTypeDef *hspi)
{
		uint16_t tmp;
		//float temprature=0;
		//MAX6675_CS(1,hspi);
		MAX6675_CS(0,hspi);
		tmp = spi_read_write_byte(0XFF,hspi); //read MSB
		tmp <<= 8;
		tmp |= spi_read_write_byte(0XFF,hspi); //read LSB
		//vTaskDelay(1);
		MAX6675_CS(1,hspi);
		if(tmp!=0)
		{
				if (tmp & 0x04) 
				{
					// thermocouple open
					tmp = 4095; //δ��⵽�ȵ�ż
					//printf("δ��⵽�ȵ�ż�����顣\r\n");
				}
				else 
				{
					tmp = tmp >> 3;
					//temprature = tmp*0.25;
					//printf("ԭʼ�����ǣ�%04X,  ��ǰ�¶��ǣ�%4.2f��\r\n",tmp,temprature);
				}
				//return temprature;
				return tmp;
		}
		else printf("max6675û�����ݷ��أ�����max6675���ӡ�\r\n");
		return tmp;
}

int MAX6675_ReadTemprature_FILTER(SPI_HandleTypeDef *hspi){  
        int temp[5]={0};
				float temprature = 0;
				int *p=temp;
				for(int i = 0;i < 5;i++){
						temp[i] = MAX6675_ReadTemprature(hspi);
             printf("the arr[%d] is :%d\n",i,temp[i]);
						vTaskDelay(200);
        }
				quickSort(p,0,4);//��ֵ�˲�
				printf("the arr[2] = %d\n",temp[2]);
				if(temp[2]==4095) {
						printf("δ��⵽�ȵ�ż�����顣\r\n");
						return 0;
				}
				else{
						temprature = temp[2]*0.25;
						printf("��ֵ�˲���ԭʼ�����ǣ�%04X,�˲����¶��ǣ�%4.2f��\r\n",temp[2],temprature);
				}
        return temprature;
}

static void quickSort(int *nums, int start, int end) {
    //�����ж��Ԫ�ؽ�������
    if (start < end) {
        int base = nums[start];//��Ҫ�������������0��Ԫ��Ϊbase
        int left = start;//��ָ��
        int right = end;//��ָ��
        while (left < right) {
            //���������ң���base��right--
            while (left< right && nums[right] >= base) {
                right--;
            }
            //��baseС���滻left����λ�õ�����
            nums[left] = nums[right];
            //���������ң���baseС��left++
            while (left < right && nums[left] <= base){
                left++;
            }
            //��base���滻right����λ�õ�����
            nums[right] = nums[left];
        }
        nums[left] = base;//��ʱleft=right����base�滻���λ�õ�����
        //���б�baseС�����ֵ�����
        quickSort(nums, start, left - 1);
        //���б�base������ֵ�����
        quickSort(nums, left + 1, end);
    }
}

/**
 * @brief       SPI1�ٶ����ú���
 *   @note      SPI1ʱ��ѡ������APB1, ��PCLK1, Ϊ 42MHz
 *              SPI�ٶ� = PCLK1 / 2^(speed + 1)
 * @param       speed   : SPI1ʱ�ӷ�Ƶϵ��
                        ȡֵΪSPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
 * @retval      ��
 */
void spi1_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed)); /* �ж���Ч�� */
    __HAL_SPI_DISABLE(&g_spi1_handler);             /* �ر�SPI */
    g_spi1_handler.Instance->CR1 &= 0XFFC7;         /* λ3-5���㣬�������ò����� */
    g_spi1_handler.Instance->CR1 |= speed << 3;     /* ����SPI�ٶ� */
    __HAL_SPI_ENABLE(&g_spi1_handler);              /* ʹ��SPI */
}

/**
 * @brief       SPI1��дһ���ֽ�����
 * @param       txdata  : Ҫ���͵�����(1�ֽ�)
 * @retval      ���յ�������(1�ֽ�)
 */
uint8_t spi_read_write_byte(uint8_t txdata,SPI_HandleTypeDef *hspi)
{
    uint8_t rxdata;
    HAL_SPI_TransmitReceive(hspi, &txdata, &rxdata, 1, 1000);
    return rxdata; /* �����յ������� */
}


//uint8_t spi2_read_write_byte(uint8_t txdata)
//{
//    uint8_t rxdata;
//    HAL_SPI_TransmitReceive(&g_spi2_handler, &txdata, &rxdata, 1, 1000);
//    return rxdata; /* �����յ������� */
//}
//MAX6675 Ƭѡ����
void MAX6675_CS(unsigned char choose,SPI_HandleTypeDef *hspi)
{
		if (hspi->Instance == SPI1_SPI)
	{
			if(choose == 1)
			{
				HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT, SPI1_CS_GPIO_PIN, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT, SPI1_CS_GPIO_PIN, GPIO_PIN_RESET);
			}
	}
	
		if (hspi->Instance == SPI2_SPI)
	{
			if(choose == 1)
			{
				HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_GPIO_PIN, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_GPIO_PIN, GPIO_PIN_RESET);
			}
	}
}


