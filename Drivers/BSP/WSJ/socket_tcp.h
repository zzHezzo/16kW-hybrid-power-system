#ifndef LWIP_TCPECHO_H
#define LWIP_TCPECHO_H
#include "./BSP/LED/led.h"

void tcpecho_init(void);
static void vTaskTaskUserIF(char *recv_data);
void string_to_u16(uint16_t *data, uint8_t *buf, uint8_t location);
//typedef struct Msg_R
//{
//		uint16_t battery_status,Electronic_Speed_Controller_relay,rectifier_relay;
//	
//}MSG_R;
#endif /* LWIP_TCPECHO_H */
