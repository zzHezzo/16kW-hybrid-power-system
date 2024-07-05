#include "zzh.h"
#include "FreeRTOS.h"

//90бу750-1 180бу1250-1
volatile uint16_t valve_servo = 0, PWM_relay = 18000,
	battery_status = 1, Electronic_Speed_Controller_relay = 1, rectifier_relay = 1, 
	first_cylinder_temprature = 0, second_cylinder_temprature = 0,
	engine_working_state=0, engine_speed=0, engine_warm_up=0, engine_idling_set=0,
	/*		10					12				18				24	*/
	
	air_inflow=0, throttle_position=0;
	/*	28				74		*/
volatile char tcp_server_sendbuf[64];

void u16_to_string(uint16_t data, uint8_t *buf, uint8_t location){
	buf[location] = data;
	buf[location+1] = data>>8;
}

void string_to_u16(uint16_t *data, uint8_t *buf, uint8_t location){
	*data = buf[location+1];
	*data = *data<<8&0XFF00;
	*data += buf[location];
}

void recv_data_from_PC(char* buf)
{
			string_to_u16(&battery_status,(uint8_t*)buf,4);      //printf("battery_status %d\n",battery_status);
			string_to_u16(&valve_servo,(uint8_t*)buf,10);					//printf("valve_servo %d\n",valve_servo);
			string_to_u16(&Electronic_Speed_Controller_relay,(uint8_t*)buf,12);  	//printf("Electronic_Speed_Controller_relay %d\n",Electronic_Speed_Controller_relay);
			string_to_u16(&rectifier_relay,(uint8_t*)buf,14);			//printf("rectifier_relay %d\n",rectifier_relay);
}

char* send_data_to_pc(void){

	tcp_server_sendbuf[0] = sizeof(tcp_server_sendbuf);	
	tcp_server_sendbuf[1] = 103;
	tcp_server_sendbuf[63] = 104;
	
	//read_cylinder_temprature();
	u16_to_string(first_cylinder_temprature,(uint8_t*)tcp_server_sendbuf, 4);
	u16_to_string(second_cylinder_temprature,(uint8_t*)tcp_server_sendbuf, 6);
	
	u16_to_string(engine_working_state,(uint8_t*)tcp_server_sendbuf, 8);
	u16_to_string(engine_speed,(uint8_t*)tcp_server_sendbuf, 10);
	u16_to_string(engine_warm_up,(uint8_t*)tcp_server_sendbuf, 12);
	u16_to_string(engine_idling_set,(uint8_t*)tcp_server_sendbuf, 14);
	u16_to_string(air_inflow,(uint8_t*)tcp_server_sendbuf, 16);
	u16_to_string(throttle_position,(uint8_t*)tcp_server_sendbuf, 18);
	return (char*)tcp_server_sendbuf;
}

