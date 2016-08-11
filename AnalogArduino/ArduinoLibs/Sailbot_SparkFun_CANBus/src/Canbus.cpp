/**
 * 
 *
 * Copyright (c) 2008-2009  All rights reserved.
 */

#if ARDUINO>=100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include <stdint.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins_arduino.h"
#include <inttypes.h>
#include "global.h"
#include "mcp2515.h"
#include "defaults.h"
#include "Canbus.h"




/* C++ wrapper */
CanbusClass::CanbusClass() {

 
}
char CanbusClass::message_rx(unsigned char *buffer,unsigned char *rtr, unsigned char *length, unsigned long *id) {
		tCAN message;
	
		if (mcp2515_check_message()) {
		
			
			// Lese die Nachricht aus dem Puffern des MCP2515
			if (mcp2515_get_message(&message)) {
			//	print_can_message(&message);
			//	PRINT("\n");
				buffer[0] = message.data[0];
				buffer[1] = message.data[1];
				buffer[2] = message.data[2];
				buffer[3] = message.data[3];
				buffer[4] = message.data[4];
				buffer[5] = message.data[5];
				buffer[6] = message.data[6];
				buffer[7] = message.data[7];
				*id = message.id;
				*rtr = message.header.rtr;
				*length=message.header.length;
				return 1; //SUCCESS
			}
			else {
			//	PRINT("Kann die Nachricht nicht auslesen\n\n");
				return -1; //ERROR
			}
		}
		
		return 0;

}

char CanbusClass::message_tx(unsigned char m) {
	tCAN message;


	// einige Testwerte
	message.id = 0x7DF;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = 0x00;
	message.data[1] = 0x00;
	message.data[2] = m;
	message.data[3] = 0x00;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	
	
	
	
//	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));	
		mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		
	if (mcp2515_send_message(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}

char CanbusClass::message_NMT(unsigned char id, unsigned char CCD) {
	tCAN message;


	// einige Testwerte
	message.id = 0x000;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = CCD;
	message.data[1] = id;
	message.data[2] = 0x00;
	message.data[3] = 0x00;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	
	
	
	
//	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));	
		mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		
	if (mcp2515_send_message(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}

char CanbusClass::start_node(unsigned char id) {
	message_NMT(id, NMT_START);
}
char CanbusClass::stop_node(unsigned char id) {
	message_NMT(id, NMT_STOP);
}
char CanbusClass::preop_node(unsigned char id) {
	message_NMT(id, NMT_PREOP);
}
char CanbusClass::reset_node(unsigned char id) {
	message_NMT(id, NMT_RESET);
}
char CanbusClass::reset_comm_node(unsigned char id) {
	message_NMT(id, NMT_RECOM);
}

char CanbusClass::message_LSS(unsigned char CS, unsigned char byte1, unsigned char byte2, unsigned char byte3) {
	tCAN message;


	// einige Testwerte
	message.id = LSS_MASTER2SLAVE;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = CS;
	message.data[1] = byte1;
	message.data[2] = byte2;
	message.data[3] = byte3;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	
	
	
	
//	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));	
		mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		
	if (mcp2515_send_message(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}

char CanbusClass::switch_lss_config(unsigned char onOff) {
	message_LSS(LSS_SWITCH_CONFIG,onOff,BLANK,BLANK);
}

char CanbusClass::change_id(unsigned char new_id) {
	message_LSS(LSS_CHANGE_ID,new_id,BLANK,BLANK);
}

char CanbusClass::change_baud_rate(unsigned char new_rate) {
	message_LSS(LSS_CHANGE_BAUD,new_rate,BLANK,BLANK);
}

char CanbusClass::save_lss_config() {
	message_LSS(LSS_SAVE,BLANK,BLANK,BLANK);
}

char CanbusClass::get_node_id() {
	message_LSS(LSS_INQUIRE_NODE,BLANK,BLANK,BLANK);
}
char CanbusClass::get_serial_num() {
	message_LSS(LSS_INQUIRE_SERIAL,BLANK,BLANK,BLANK);
}
char CanbusClass::get_revision_num() {
	message_LSS(LSS_INQUIRE_REVIS,BLANK,BLANK,BLANK);
}
char CanbusClass::get_product_code() {
	message_LSS(LSS_INQUIRE_PROD,BLANK,BLANK,BLANK);
}
char CanbusClass::get_vendor_id() {
	message_LSS(LSS_INQUIRE_VENDOR,BLANK,BLANK,BLANK);
}

char CanbusClass::identify_non_configured() {
	message_LSS(LSS_NON_CONFIG,BLANK,BLANK,BLANK);
}

char CanbusClass::message_SDO(unsigned char id, unsigned char CCD, unsigned short index, unsigned char sub_index, unsigned int data) {
	tCAN message;


	// einige Testwerte
	message.id = SEND_SDO+id;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = CCD;
	message.data[1] = index & 0xFF;
	Serial.print(message.data[1]);
	message.data[2] = (index >> 8) & 0xFF;
	message.data[3] = sub_index;
	message.data[4] = data & 0xFF;
	message.data[5] = (data >> 8) & 0xFF;
	message.data[6] = (data >> 16) & 0xFF;
	message.data[7] = (data >> 24) & 0xFF;		
	
	
	
	
//	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));	
		mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		
	if (mcp2515_send_message(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}


char CanbusClass::read_heartbeat(unsigned char id){
	message_SDO(id,SDO_READ,OL_HEARTBEAT,BLANK,BLANK);
}

char CanbusClass::write_heartbeat(unsigned char id, unsigned int data){
	message_SDO(id,SDO_WRITE,OL_HEARTBEAT,BLANK,data);
}

char CanbusClass::read_pos(unsigned char id){
	message_SDO(id,SDO_READ,OL_POS,BLANK,BLANK);
}

char CanbusClass::write_pos(unsigned char id, unsigned int data){
	message_SDO(id,SDO_WRITE,OL_POS,BLANK,data);
}

char CanbusClass::read_num(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_NUM,BLANK);
}

char CanbusClass::write_num(unsigned char id, unsigned char pdo_num, unsigned int new_num){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_NUM,new_num);
}

char CanbusClass::read_cob(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_COB,BLANK);
}

char CanbusClass::write_cob(unsigned char id, unsigned char pdo_num, unsigned int new_cob){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_COB,new_cob);
}

char CanbusClass::read_tx_type(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_TXTYPE,BLANK);
}

char CanbusClass::write_tx_type(unsigned char id, unsigned char pdo_num, unsigned int new_tx_type){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_TXTYPE,new_tx_type);
}

char CanbusClass::read_inhibition_time(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_INHTIME,BLANK);
}

char CanbusClass::write_inhibition_time(unsigned char id, unsigned char pdo_num, unsigned int new_inhibition_time){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_INHTIME,new_inhibition_time);
}

char CanbusClass::read_event_timer(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_EVNTTMR,BLANK);
}

char CanbusClass::write_event_timer(unsigned char id, unsigned char pdo_num, unsigned int new_event_timer){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_EVNTTMR,new_event_timer);
}

char CanbusClass::ecu_req(unsigned char pid,  char *buffer) 
{
	tCAN message;
	float engine_data;
	int timeout = 0;
	char message_ok = 0;
	// Prepair message
	message.id = PID_REQUEST;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = 0x02;
	message.data[1] = 0x01;
	message.data[2] = pid;
	message.data[3] = 0x00;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	

	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
	
	if (mcp2515_send_message(&message)) {
	}
				if (mcp2515_check_message()) 
				{

					if (mcp2515_get_message(&message)) 
					{
							switch(message.data[2])
								{   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
									case ENGINE_RPM:  			//   ((A*256)+B)/4    [RPM]
									engine_data =  ((message.data[3]*256) + message.data[4])/4;
									sprintf(buffer,"%d rpm ",(int) engine_data);
									break;
							
									case ENGINE_COOLANT_TEMP: 	// 	A-40			  [degree C]
									engine_data =  message.data[3] - 40;
									sprintf(buffer,"%d degC",(int) engine_data);
									break;
							
									case VEHICLE_SPEED: 		// A				  [km]
									engine_data =  message.data[3];
									sprintf(buffer,"%d km ",(int) engine_data);
									break;

									case MAF_SENSOR:   			// ((256*A)+B) / 100  [g/s]
									engine_data =  ((message.data[3]*256) + message.data[4])/100;
									sprintf(buffer,"%d g/s",(int) engine_data);
									break;

									case O2_VOLTAGE:    		// A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
									engine_data = message.data[3]*0.005;
									sprintf(buffer,"%d V",(int) engine_data);
									break;
									
									case THROTTLE:				// Throttle Position
									engine_data = (message.data[3]*100)/255;
									sprintf(buffer,"%d %% ",(int) engine_data);
									break;
							
								}
					}
				}

}






char CanbusClass::init(unsigned char speed) {

  return mcp2515_init(speed);
 
}

CanbusClass Canbus;
