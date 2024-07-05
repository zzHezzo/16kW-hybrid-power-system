#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 引脚 定义 */
#define LED0_GPIO_PORT                  GPIOF
#define LED0_GPIO_PIN                   GPIO_PIN_9
#define LED0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)             /* PF口时钟使能 */

#define LED1_GPIO_PORT                  GPIOF
#define LED1_GPIO_PIN                   GPIO_PIN_10
#define LED1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)             /* PF口时钟使能 */

#define engine_relay_GPIO_PORT                  GPIOE
#define engine_relay_GPIO_PIN                   GPIO_PIN_5																							//电源继电器
#define engine_relay_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)             /* PE口时钟使能 */

#define motor_relay_GPIO_PORT                  GPIOE
#define motor_relay_GPIO_PIN                   GPIO_PIN_2																								//电调继电器
#define motor_relay_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)             /* PE口时钟使能 */

#define rectifier_relay_GPIO_PORT                  GPIOE
#define rectifier_relay_GPIO_PIN                   GPIO_PIN_3																							//整流器继电器
#define rectifier_relay_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)             /* PE口时钟使能 */

/******************************************************************************************/
/* LED端口定义 */
#define LED0(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED0 = RED */

#define LED1(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)       /* LED1 = GREEN */

#define Engine_relay(x)   do{ x ? \
                      HAL_GPIO_WritePin(engine_relay_GPIO_PORT, engine_relay_GPIO_PIN, GPIO_PIN_RESET) : \
                      HAL_GPIO_WritePin(engine_relay_GPIO_PORT, engine_relay_GPIO_PIN, GPIO_PIN_SET); \
                  }while(0)       

#define Motor_relay(x)   do{ x ? \
                      HAL_GPIO_WritePin(motor_relay_GPIO_PORT, motor_relay_GPIO_PIN, GPIO_PIN_RESET) : \
                      HAL_GPIO_WritePin(motor_relay_GPIO_PORT, motor_relay_GPIO_PIN, GPIO_PIN_SET); \
                  }while(0)       

#define Rectifier_relay(x)   do{ x ? \
                      HAL_GPIO_WritePin(rectifier_relay_GPIO_PORT, rectifier_relay_GPIO_PIN, GPIO_PIN_RESET) : \
                      HAL_GPIO_WritePin(rectifier_relay_GPIO_PORT, rectifier_relay_GPIO_PIN, GPIO_PIN_SET); \
                  }while(0)       

/******************************************************************************************/
/* LED取反定义 */
#define LED0_TOGGLE()    do{ HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_GPIO_PIN); }while(0)       /* LED0 = !LED0 */
#define LED1_TOGGLE()    do{ HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); }while(0)       /* LED1 = !LED1 */
#define engine_relay_TOGGLE()    do{ HAL_GPIO_TogglePin(engine_relay_GPIO_PORT, engine_relay_GPIO_PIN); }while(0)       /* LED1 = !LED1 */
#define motor_relay_TOGGLE()    do{ HAL_GPIO_TogglePin(motor_relay_GPIO_PORT, motor_relay_GPIO_PIN); }while(0)       /* LED1 = !LED1 */
#define rectifier_relay_TOGGLE()    do{ HAL_GPIO_TogglePin(rectifier_relay_GPIO_PORT, rectifier_relay_GPIO_PIN); }while(0)       /* LED1 = !LED1 */

/******************************************************************************************/
/* 外部接口函数*/
void led_init(void);                                                                            /* 初始化 */

#endif
