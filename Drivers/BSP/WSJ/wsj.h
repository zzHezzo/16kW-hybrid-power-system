#ifndef _WSJ_H
#define _WSJ_H
#include "sys.h"

void safety_protection(void);
void pc_data_handle(u8 *buf);
void send_to_pc(void);
void ECU_data_recv(void);
void ECU_data_ask(void);
void relay_status(void);
extern void electric_relay_Init(void);
void string_to_u16(u16 *data, u8 *buf, u8 location);
void u16_to_string(u16 data, u8 *buf, u8 location);
void float_to_string(float data, u8 *buf, u8 location);

extern u16 battery_status, valve_servo, Electronic_Speed_Controller_relay, rectifier_relay, PWM_relay,
		   first_cylinder_temprature, second_cylinder_temprature;
#endif
