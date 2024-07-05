#include "./BSP/SPI/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"

SPI_HandleTypeDef g_spi1_handler; /* SPI1句柄 */
SPI_HandleTypeDef g_spi2_handler; /* SPI2句柄 */

/**
 * @brief       MAX6675初始化代码
 *   @note      主机模式,8位数据,禁止硬件片选
 * @param       无
 * @retval      无
 */
void spi1_MAX6675_init(void)
{
    SPI1_SPI_CLK_ENABLE(); /* SPI1时钟使能 */

    g_spi1_handler.Instance = SPI1_SPI;                                /* SPI1 */
    g_spi1_handler.Init.Mode = SPI_MODE_MASTER;                        /* 设置SPI工作模式，设置为主模式 */
    g_spi1_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* 设置SPI单向或者双向的数据模式:SPI设置为双线模式 */
    g_spi1_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* 设置SPI的数据大小:SPI发送接收8位帧结构 */
    g_spi1_handler.Init.CLKPolarity = SPI_POLARITY_LOW;                /* 串行同步时钟的空闲状态为低电平 */
    g_spi1_handler.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* 串行同步时钟的第一个跳变沿（上升或下降）数据被采样 */
    g_spi1_handler.Init.NSS = SPI_NSS_SOFT;                            /* NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制 */
    g_spi1_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  /* 定义波特率预分频的值:波特率预分频值为16 MAX6675工作在4.3MHz APB2 84MHz*/
    g_spi1_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始 */
    g_spi1_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* 关闭TI模式 */
    g_spi1_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* 关闭硬件CRC校验 */
    g_spi1_handler.Init.CRCPolynomial = 7;                             /* CRC值计算的多项式 */
    HAL_SPI_Init(&g_spi1_handler);                                     /* 初始化 */

    __HAL_SPI_ENABLE(&g_spi1_handler); /* 使能SPI1 */

    //spi_read_write_byte(0XFF,&g_spi1_handler); /* 启动传输, 实际上就是产生8个时钟脉冲, 达到清空DR的作用, 非必需 */
}

void spi2_MAX6675_init(void)
{
    SPI2_SPI_CLK_ENABLE(); /* SPI2时钟使能 */

    g_spi2_handler.Instance = SPI2_SPI;                                /* SPI2 */
    g_spi2_handler.Init.Mode = SPI_MODE_MASTER;                        /* 设置SPI工作模式，设置为主模式 */
    g_spi2_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* 设置SPI单向或者双向的数据模式:SPI设置为双线模式 */
    g_spi2_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* 设置SPI的数据大小:SPI发送接收8位帧结构 */
    g_spi2_handler.Init.CLKPolarity = SPI_POLARITY_LOW;                /* 串行同步时钟的空闲状态为低电平 */
    g_spi2_handler.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* 串行同步时钟的第一个跳变沿（上升或下降）数据被采样 */
    g_spi2_handler.Init.NSS = SPI_NSS_SOFT;                            /* NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制 */
    g_spi2_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; 	 /* 定义波特率预分频的值:波特率预分频值为8 spi2挂载APB1 42MHz */
    g_spi2_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始 */
    g_spi2_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* 关闭TI模式 */
    g_spi2_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* 关闭硬件CRC校验 */
    g_spi2_handler.Init.CRCPolynomial = 7;                             /* CRC值计算的多项式 */
    HAL_SPI_Init(&g_spi2_handler);                                     /* 初始化 */

    __HAL_SPI_ENABLE(&g_spi2_handler); /* 使能SPI2 */

    //spi_read_write_byte(0XFF,&g_spi2_handler); /* 启动传输, 实际上就是产生8个时钟脉冲, 达到清空DR的作用, 非必需 */
}


/**
 * @brief       SPI初始化代码
 *   @note      主机模式,8位数据,禁止硬件片选
 * @param       无
 * @retval      无
 */
//void spi1_init(void)
//{
//    SPI1_SPI_CLK_ENABLE(); /* SPI1时钟使能 */

//    g_spi1_handler.Instance = SPI1_SPI;                                /* SPI1 */
//    g_spi1_handler.Init.Mode = SPI_MODE_MASTER;                        /* 设置SPI工作模式，设置为主模式 */
//    g_spi1_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* 设置SPI单向或者双向的数据模式:SPI设置为双线模式 */
//    g_spi1_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* 设置SPI的数据大小:SPI发送接收8位帧结构 */
//    g_spi1_handler.Init.CLKPolarity = SPI_POLARITY_LOW;               /* 串行同步时钟的空闲状态为低电平 */
//    g_spi1_handler.Init.CLKPhase = SPI_PHASE_1EDGE;                    /* 串行同步时钟的第一个跳变沿（上升或下降）数据被采样 */
//    g_spi1_handler.Init.NSS = SPI_NSS_SOFT;                            /* NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制 */
//    g_spi1_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* 定义波特率预分频的值:波特率预分频值为256 */
//    g_spi1_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始 */
//    g_spi1_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* 关闭TI模式 */
//    g_spi1_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* 关闭硬件CRC校验 */
//    g_spi1_handler.Init.CRCPolynomial = 7;                             /* CRC值计算的多项式 */
//    HAL_SPI_Init(&g_spi1_handler);                                     /* 初始化 */

//    __HAL_SPI_ENABLE(&g_spi1_handler); /* 使能SPI1 */

//    spi1_read_write_byte(0XFF); /* 启动传输, 实际上就是产生8个时钟脉冲, 达到清空DR的作用, 非必需 */
//}

/**
 * @brief       SPI1底层驱动，时钟使能，引脚配置
 *   @note      此函数会被HAL_SPI_Init()调用
 * @param       hspi:SPI句柄
 * @retval      无
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_init_struct;
    if (hspi->Instance == SPI1_SPI)
    {
				SPI1_CS_GPIO_CLK_ENABLE();	/* SPI1_CS脚时钟使能 */
        SPI1_SCK_GPIO_CLK_ENABLE();  /* SPI1_SCK脚时钟使能 */
        SPI1_MISO_GPIO_CLK_ENABLE(); /* SPI1_MISO脚时钟使能 */
        SPI1_MOSI_GPIO_CLK_ENABLE(); /* SPI1_MOSI脚时钟使能 */
			
        /* SCK引脚模式设置(复用输出) */
        gpio_init_struct.Pin = SPI1_SCK_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(SPI1_SCK_GPIO_PORT, &gpio_init_struct);

			  /* MOSI引脚模式设置(复用输出) */
        gpio_init_struct.Pin = SPI1_MOSI_GPIO_PIN;
        HAL_GPIO_Init(SPI1_MOSI_GPIO_PORT, &gpio_init_struct);
        
				/* MISO引脚模式设置(复用输出) */
				//gpio_init_struct.Mode = GPIO_MODE_INPUT;
        gpio_init_struct.Pin = SPI1_MISO_GPIO_PIN;
        HAL_GPIO_Init(SPI1_MISO_GPIO_PORT, &gpio_init_struct);	

				/*片选初始化 PA4*/
				gpio_init_struct.Pin = SPI1_CS_GPIO_PIN;
				gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
				//gpio_init_struct.Pull = GPIO_PULLUP;
				gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
				HAL_GPIO_Init(SPI1_CS_GPIO_PORT, &gpio_init_struct);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_PORT,SPI1_CS_GPIO_PIN,GPIO_PIN_SET);//先把片选拉高
    }
		
    if (hspi->Instance == SPI2_SPI)
    {
				SPI2_CS_GPIO_CLK_ENABLE();	/* SPI2_CS脚时钟使能 */
        SPI2_SCK_GPIO_CLK_ENABLE();  /* SPI2_SCK脚时钟使能 */
        SPI2_MISO_GPIO_CLK_ENABLE(); /* SPI2_MISO脚时钟使能 */
        SPI2_MOSI_GPIO_CLK_ENABLE(); /* SPI2_MOSI脚时钟使能 */

        /* SCK引脚模式设置(复用输出) */
        gpio_init_struct.Pin = SPI2_SCK_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(SPI2_SCK_GPIO_PORT, &gpio_init_struct);

			  /* MOSI引脚模式设置(复用输出) */
        gpio_init_struct.Pin = SPI2_MOSI_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MOSI_GPIO_PORT, &gpio_init_struct);
        
				/* MISO引脚模式设置(复用输出) */
        gpio_init_struct.Pin = SPI2_MISO_GPIO_PIN;
        HAL_GPIO_Init(SPI2_MISO_GPIO_PORT, &gpio_init_struct);
			
				/*片选初始化 PB12*/
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
					tmp = 4095; //未检测到热电偶
					//printf("未检测到热电偶，请检查。\r\n");
				}
				else 
				{
					tmp = tmp >> 3;
					//temprature = tmp*0.25;
					//printf("原始数据是：%04X,  当前温度是：%4.2f。\r\n",tmp,temprature);
				}
				//return temprature;
				return tmp;
		}
		else printf("max6675没有数据返回，请检查max6675连接。\r\n");
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
				quickSort(p,0,4);//中值滤波
				printf("the arr[2] = %d\n",temp[2]);
				if(temp[2]==4095) {
						printf("未检测到热电偶，请检查。\r\n");
						return 0;
				}
				else{
						temprature = temp[2]*0.25;
						printf("中值滤波后原始数据是：%04X,滤波后温度是：%4.2f。\r\n",temp[2],temprature);
				}
        return temprature;
}

static void quickSort(int *nums, int start, int end) {
    //数组有多个元素进行排序
    if (start < end) {
        int base = nums[start];//以要进行排序数组第0个元素为base
        int left = start;//左指针
        int right = end;//右指针
        while (left < right) {
            //从右向左找，比base大，right--
            while (left< right && nums[right] >= base) {
                right--;
            }
            //比base小，替换left所在位置的数字
            nums[left] = nums[right];
            //从左向右找，比base小，left++
            while (left < right && nums[left] <= base){
                left++;
            }
            //比base大，替换right所在位置的数字
            nums[right] = nums[left];
        }
        nums[left] = base;//此时left=right，用base替换这个位置的数字
        //排列比base小的数字的数组
        quickSort(nums, start, left - 1);
        //排列比base大的数字的数组
        quickSort(nums, left + 1, end);
    }
}

/**
 * @brief       SPI1速度设置函数
 *   @note      SPI1时钟选择来自APB1, 即PCLK1, 为 42MHz
 *              SPI速度 = PCLK1 / 2^(speed + 1)
 * @param       speed   : SPI1时钟分频系数
                        取值为SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
 * @retval      无
 */
void spi1_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed)); /* 判断有效性 */
    __HAL_SPI_DISABLE(&g_spi1_handler);             /* 关闭SPI */
    g_spi1_handler.Instance->CR1 &= 0XFFC7;         /* 位3-5清零，用来设置波特率 */
    g_spi1_handler.Instance->CR1 |= speed << 3;     /* 设置SPI速度 */
    __HAL_SPI_ENABLE(&g_spi1_handler);              /* 使能SPI */
}

/**
 * @brief       SPI1读写一个字节数据
 * @param       txdata  : 要发送的数据(1字节)
 * @retval      接收到的数据(1字节)
 */
uint8_t spi_read_write_byte(uint8_t txdata,SPI_HandleTypeDef *hspi)
{
    uint8_t rxdata;
    HAL_SPI_TransmitReceive(hspi, &txdata, &rxdata, 1, 1000);
    return rxdata; /* 返回收到的数据 */
}


//uint8_t spi2_read_write_byte(uint8_t txdata)
//{
//    uint8_t rxdata;
//    HAL_SPI_TransmitReceive(&g_spi2_handler, &txdata, &rxdata, 1, 1000);
//    return rxdata; /* 返回收到的数据 */
//}
//MAX6675 片选控制
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


