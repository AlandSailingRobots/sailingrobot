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
#include "mcp_can_dfs.h"
#include "mcp_CAN.h"
#include "defaults.h"
#include "AHM36A.h"

#ifndef SPI_CS_PIN
#define SPI_CS_PIN 10
#endif


/* C++ wrapper */
EncoderClass::EncoderClass() {
}

char EncoderClass::message_tx(unsigned char m) {
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
		
	if (tCAN_tx(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}


char EncoderClass::tCAN_tx(tCAN *message) {
	CAN->sendMsgBuf(message->id, 0, message->header.rtr, message->header.length, message->data);
}

char EncoderClass::message_NMT(unsigned char id, unsigned char CCD) {
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
	
	
	
	
	if (tCAN_tx(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}

char EncoderClass::start_node(unsigned char id) {
	message_NMT(id, NMT_START);
}
char EncoderClass::stop_node(unsigned char id) {
	message_NMT(id, NMT_STOP);
}
char EncoderClass::preop_node(unsigned char id) {
	message_NMT(id, NMT_PREOP);
}
char EncoderClass::reset_node(unsigned char id) {
	message_NMT(id, NMT_RESET);
}
char EncoderClass::reset_comm_node(unsigned char id) {
	message_NMT(id, NMT_RECOM);
}

char EncoderClass::message_LSS(unsigned char CS, unsigned char byte1, unsigned char byte2, unsigned char byte3) {
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
	
	
	
	
	if (tCAN_tx(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}

char EncoderClass::switch_lss_config(unsigned char onOff) {
	message_LSS(LSS_SWITCH_CONFIG,onOff,BLANK,BLANK);
}

char EncoderClass::change_id(unsigned char new_id) {
	message_LSS(LSS_CHANGE_ID,new_id,BLANK,BLANK);
}

char EncoderClass::change_baud_rate(unsigned char new_rate) {
	message_LSS(LSS_CHANGE_BAUD,BLANK,new_rate,BLANK);
}

char EncoderClass::save_lss_config() {
	message_LSS(LSS_SAVE,BLANK,BLANK,BLANK);
}

char EncoderClass::get_node_id() {
	message_LSS(LSS_INQUIRE_NODE,BLANK,BLANK,BLANK);
}
char EncoderClass::get_serial_num() {
	message_LSS(LSS_INQUIRE_SERIAL,BLANK,BLANK,BLANK);
}
char EncoderClass::get_revision_num() {
	message_LSS(LSS_INQUIRE_REVIS,BLANK,BLANK,BLANK);
}
char EncoderClass::get_product_code() {
	message_LSS(LSS_INQUIRE_PROD,BLANK,BLANK,BLANK);
}
char EncoderClass::get_vendor_id() {
	message_LSS(LSS_INQUIRE_VENDOR,BLANK,BLANK,BLANK);
}

char EncoderClass::identify_non_configured() {
	message_LSS(LSS_NON_CONFIG,BLANK,BLANK,BLANK);
}

char EncoderClass::message_SDO(unsigned char id, unsigned char CCD, unsigned short index, unsigned char sub_index, unsigned int data) {
	tCAN message;


	// einige Testwerte
	message.id = SEND_SDO+id;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = CCD;
	message.data[1] = index & 0xFF;
	message.data[2] = (index >> 8) & 0xFF;
	message.data[3] = sub_index;
	message.data[4] = data & 0xFF;
	message.data[5] = (data >> 8) & 0xFF;
	message.data[6] = (data >> 16) & 0xFF;
	message.data[7] = (data >> 24) & 0xFF;		
	
	
	
	
	if (tCAN_tx(&message)) {
		//	SUCCESS;
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}


char EncoderClass::read_heartbeat(unsigned char id){
	message_SDO(id,SDO_READ,OL_HEARTBEAT,BLANK,BLANK);
}

char EncoderClass::write_heartbeat(unsigned char id, unsigned int data){
	message_SDO(id,SDO_WRITE,OL_HEARTBEAT,BLANK,data);
}

char EncoderClass::read_pos(unsigned char id){
	message_SDO(id,SDO_READ,OL_POS,BLANK,BLANK);
}

char EncoderClass::write_pos(unsigned char id, unsigned int data){
	message_SDO(id,SDO_WRITE,OL_POS,BLANK,data);
}

char EncoderClass::read_num(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_NUM,BLANK);
}

char EncoderClass::write_num(unsigned char id, unsigned char pdo_num, unsigned int new_num){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_NUM,new_num);
}

char EncoderClass::read_cob(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_COB,BLANK);
}

char EncoderClass::write_cob(unsigned char id, unsigned char pdo_num, unsigned int new_cob){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_COB,new_cob);
}

char EncoderClass::read_tx_type(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_TXTYPE,BLANK);
}

char EncoderClass::write_tx_type(unsigned char id, unsigned char pdo_num, unsigned int new_tx_type){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_TXTYPE,new_tx_type);
}

char EncoderClass::read_inhibition_time(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_INHTIME,BLANK);
}

char EncoderClass::write_inhibition_time(unsigned char id, unsigned char pdo_num, unsigned int new_inhibition_time){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_INHTIME,new_inhibition_time);
}

char EncoderClass::read_event_timer(unsigned char id, unsigned char pdo_num){
	message_SDO(id,SDO_READ, OL_FIRST_PDO+pdo_num-1,SI_PDO_EVNTTMR,BLANK);
}

char EncoderClass::write_event_timer(unsigned char id, unsigned char pdo_num, unsigned int new_event_timer){
	message_SDO(id,SDO_WRITE, OL_FIRST_PDO+pdo_num-1,SI_PDO_EVNTTMR,new_event_timer);
}

char EncoderClass::init(MCP_CAN* _CAN) {
	CAN = _CAN;
  return 1;
}

EncoderClass Encoder;
