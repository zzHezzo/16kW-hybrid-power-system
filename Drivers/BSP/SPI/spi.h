#ifndef __SPI_H
#define __SPI_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* SPI 引脚 定义 */
#define SPI1_CS_GPIO_PORT              	GPIOA
#define SPI1_CS_GPIO_PIN               	GPIO_PIN_4
#define SPI1_CS_GPIO_CLK_ENABLE()      	do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define SPI1_SCK_GPIO_PORT              GPIOA
#define SPI1_SCK_GPIO_PIN               GPIO_PIN_5
#define SPI1_SCK_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define SPI1_MISO_GPIO_PORT             GPIOA
#define SPI1_MISO_GPIO_PIN              GPIO_PIN_6
#define SPI1_MISO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define SPI1_MOSI_GPIO_PORT             GPIOB
#define SPI1_MOSI_GPIO_PIN              GPIO_PIN_5
#define SPI1_MOSI_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define SPI2_CS_GPIO_PORT              	GPIOB
#define SPI2_CS_GPIO_PIN               	GPIO_PIN_12
#define SPI2_CS_GPIO_CLK_ENABLE()      	do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define SPI2_SCK_GPIO_PORT              GPIOB
#define SPI2_SCK_GPIO_PIN               GPIO_PIN_13
#define SPI2_SCK_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define SPI2_MISO_GPIO_PORT             GPIOC
#define SPI2_MISO_GPIO_PIN              GPIO_PIN_2
#define SPI2_MISO_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */

#define SPI2_MOSI_GPIO_PORT             GPIOC
#define SPI2_MOSI_GPIO_PIN              GPIO_PIN_3
#define SPI2_MOSI_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */

/* SPI2相关定义 */
#define SPI1_SPI                        SPI1
#define SPI1_SPI_CLK_ENABLE()           do{ __HAL_RCC_SPI1_CLK_ENABLE(); }while(0)    /* SPI1时钟使能 */

#define SPI2_SPI                        SPI2
#define SPI2_SPI_CLK_ENABLE()           do{ __HAL_RCC_SPI2_CLK_ENABLE(); }while(0)    /* SPI2时钟使能 */

/******************************************************************************************/
extern SPI_HandleTypeDef g_spi1_handler; /* SPI1句柄 */
extern SPI_HandleTypeDef g_spi2_handler; /* SPI2句柄 */


/* SPI总线速度设置 */
#define SPI_SPEED_2         0
#define SPI_SPEED_4         1
#define SPI_SPEED_8         2
#define SPI_SPEED_16        3
#define SPI_SPEED_32        4
#define SPI_SPEED_64        5
#define SPI_SPEED_128       6
#define SPI_SPEED_256       7

void spi1_MAX6675_init(void);
void spi2_MAX6675_init(void);
//void spi1_init(void);
void spi1_set_speed(uint8_t speed);
uint8_t spi_read_write_byte(uint8_t txdata,SPI_HandleTypeDef *hspi);
//uint8_t spi2_read_write_byte(uint8_t txdata);
void MAX6675_CS(unsigned char choose,SPI_HandleTypeDef *hspi);
uint16_t MAX6675_ReadTemprature(SPI_HandleTypeDef *hspi);
static void quickSort(int *nums, int start, int end);
int MAX6675_ReadTemprature_FILTER(SPI_HandleTypeDef *hspi);
#endif
























