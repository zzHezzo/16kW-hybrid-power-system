#ifndef _ZZH_H
#define _ZZH_H
#include "FreeRTOS.h"
void string_to_u16(uint16_t *data, uint8_t *buf, uint8_t location);
void recv_data_from_PC(char* buf);
void u16_to_string(uint16_t data, uint8_t *buf, uint8_t location);
char* send_data_to_pc(void);

extern volatile uint16_t 
	engine_working_state, engine_speed, engine_warm_up, engine_idling_set,
	/*		10					         12				     18				          24	*/
	air_inflow, throttle_position;
	/*	28				     74		*/

extern volatile uint16_t valve_servo, PWM_relay,
	battery_status, Electronic_Speed_Controller_relay, rectifier_relay, 
	first_cylinder_temprature, second_cylinder_temprature;

#endif

