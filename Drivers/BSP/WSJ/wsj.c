#include "headfile.h"



u16 valve_servo = 17900, PWM_relay = 18000,
	battery_status = 0, Electronic_Speed_Controller_relay = 0, rectifier_relay = 0, 
	first_cylinder_temprature = 0, second_cylinder_temprature = 0,
	engine_working_state, engine_speed, engine_warm_up, engine_idling_set,
	/*		10					12				18				24	*/
	
	air_inflow, throttle_position;
	/*	28				74		*/

void safety_protection(void){
	if(first_cylinder_temprature >= 210 || second_cylinder_temprature >= 210)
		GPIO_ResetBits(GPIOE,GPIO_Pin_5);
}

//void pc_data_handle(u8 *buf){
//	
//	string_to_u16(&battery_status,buf,4);
//	string_to_u16(&valve_servo,buf,10);
//	string_to_u16(&Electronic_Speed_Controller_relay,buf,12);
//	string_to_u16(&rectifier_relay,buf,14);
//	
//	relay_status();
//	
//	TIM_SetCompare1(TIM14,valve_servo+17900);
//}

void send_to_pc(void){
	tcp_server_sendbuf[0] = sizeof(tcp_server_sendbuf);	
	tcp_server_sendbuf[1] = 103;
	tcp_server_sendbuf[63] = 104;
	
	read_cylinder_temprature();
	u16_to_string(first_cylinder_temprature,tcp_server_sendbuf, 4);
	u16_to_string(second_cylinder_temprature,tcp_server_sendbuf, 6);
	
	u16_to_string(engine_working_state,tcp_server_sendbuf, 8);
	u16_to_string(engine_speed,tcp_server_sendbuf, 10);
	u16_to_string(engine_warm_up,tcp_server_sendbuf, 12);
	u16_to_string(engine_idling_set,tcp_server_sendbuf, 14);
	u16_to_string(air_inflow,tcp_server_sendbuf, 16);
	u16_to_string(throttle_position,tcp_server_sendbuf, 18);
	
	tcp_server_flag|=1<<7;//标记要发送数据
}

void ECU_data_recv(void){

//	static u8 i=0;
	
	RS485_Receive_Data(recv_232buf);
	if(recv_232buf[0]){
		string_to_u16(&engine_working_state,recv_232buf,10);
		string_to_u16(&engine_speed,recv_232buf,12);
		string_to_u16(&engine_warm_up,recv_232buf,18);
		string_to_u16(&engine_idling_set,recv_232buf,24);
		string_to_u16(&air_inflow,recv_232buf,28);
		string_to_u16(&throttle_position,recv_232buf,74);
	}
}

void ECU_data_ask(void){
	send_232buf[0] = 0x03;	send_232buf[1] = 0x04;	send_232buf[2] = 0xF9;
	RS485_Send_Data(send_232buf,3);
	
//	send_232buf[0] = 0x03;	send_232buf[1] = 0x0B;	send_232buf[2] = 0xF2;
//	RS485_Send_Data(send_232buf,3);	
}

//void relay_status(void){
//	if(battery_status)	GPIO_SetBits(GPIOE,GPIO_Pin_5);	
//	else	GPIO_ResetBits(GPIOE,GPIO_Pin_5);	
//	
//	if(Electronic_Speed_Controller_relay)	GPIO_SetBits(GPIOE,GPIO_Pin_2);
//	else	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
//	
//	if(rectifier_relay)	GPIO_SetBits(GPIOE,GPIO_Pin_3);
//	else	GPIO_ResetBits(GPIOE,GPIO_Pin_3);
//}

//void electric_relay_Init(void){
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOE的时钟
//	
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
//	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
//	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;  //上拉
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
//	GPIO_Init(GPIOE,&GPIO_InitStructure);
//	
//	GPIO_ResetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5); 
//}

void string_to_u16(u16 *data, u8 *buf, u8 location){
	*data = buf[location+1];
	*data = *data<<8&0XFF00;
	*data += buf[location];
}

void u16_to_string(u16 data, u8 *buf, u8 location){
	buf[location] = data;
	buf[location+1] = data>>8;
}

void float_to_string(float data, u8 *buf, u8 location){
	u8 *pdata = (u8*)&data;
	int i = 1;
	for(i = 0; i<4; i++)
	buf[location+i] = *pdata++;
}
