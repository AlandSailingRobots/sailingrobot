/**
 * CAN BUS
 *
 * Copyright (c) 2010 Sukkin Pang All rights reserved.
 */

#ifndef canbus__h
#define canbus__h

#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps

#define BLANK				0x00

#define NMT_START			0x01
#define NMT_STOP			0x02 
#define NMT_PREOP			0x80
#define NMT_RESET			0x81
#define NMT_RECOM			0x82

#define LSS_MASTER2SLAVE	0x07E5
#define LSS_CONFIG_OFF		0x00
#define LSS_CONFIG_ON		0x01
#define LSS_SWITCH_CONFIG	0x04
#define LSS_CHANGE_ID		0x11	
#define LSS_CHANGE_BAUD		0x13
#define LSS_BAUD_1000		0x00
#define LSS_BAUD_800		0x01
#define LSS_BAUD_500		0x02
#define LSS_BAUD_250		0x03
#define LSS_BAUD_125		0x04
#define LSS_BAUD_100		0x05
#define LSS_BAUD_50			0x06
#define LSS_BAUD_20			0x07
#define LSS_BAUD_10			0x08
#define LSS_BAUD_AUTO		0x09
#define LSS_SAVE			0x17
#define LSS_INQUIRE_NODE	0x5E
#define LSS_INQUIRE_SERIAL	0x5D
#define LSS_INQUIRE_REVIS	0x5C
#define LSS_INQUIRE_PROD	0x5B
#define LSS_INQUIRE_VENDOR	0x5A
#define LSS_NON_CONFIG		0x4C

#define SEND_PDO			0x0200

#define SEND_SDO			0x0600
#define SDO_READ			0x40
#define SDO_READ_RESP		0x43
#define SDO_WRITE			0x23
#define SDO_WRITE_RESP		0x60
#define SDO_ERROR			0x80

#define OL_HEARTBEAT		0x1017
#define OL_POS				0x6004
#define OL_FIRST_PDO		0x1800
#define OL_SECOND_PDO		0x1801
#define OL_THIRD_PDO		0x1802
#define OL_FOURTH_PDO		0x1803

#define SI_PDO_NUM			0x00
#define SI_PDO_COB			0x01
#define SI_PDO_TXTYPE		0x02
#define SI_PDO_INHTIME		0x03
#define SI_PDO_RESERVED		0x04
#define SI_PDO_EVNTTMR		0x05



#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE			0x11

#define PID_REQUEST         0x7DF
#define PID_REPLY			0x7E8

class CanbusClass
{
  public:

	CanbusClass();
    char init(unsigned char);
	char message_tx(unsigned char m);
	
	char message_NMT(unsigned char id, unsigned char CCD);
	char start_node(unsigned char id);
	char stop_node(unsigned char id);
	char preop_node(unsigned char id);
	char reset_node(unsigned char id);
	char reset_comm_node(unsigned char id);
	
	char message_LSS(unsigned char CS, unsigned char byte1, unsigned char byte2, unsigned char byte3);
	char switch_lss_config(unsigned char onOff);
	char change_id(unsigned char new_id);
	char change_baud_rate(unsigned char new_rate);
	char save_lss_config();
	char get_node_id();
	char get_serial_num();
	char get_revision_num();
	char get_product_code();
	char get_vendor_id();
	char identify_non_configured();
	
	char message_SDO(unsigned char id, unsigned char CCD, unsigned short index, unsigned char sub_index, unsigned int data);
	char read_heartbeat(unsigned char id);
	char write_heartbeat(unsigned char id, unsigned int data);
	char read_pos(unsigned char id);
	char write_pos(unsigned char id, unsigned int data);
	char read_num(unsigned char id, unsigned char pdo_num);
	char write_num(unsigned char id, unsigned char pdo_num, unsigned int new_num);
	char read_cob(unsigned char id, unsigned char pdo_num);
	char write_cob(unsigned char id, unsigned char pdo_num, unsigned int new_cob);
	char read_tx_type(unsigned char id, unsigned char pdo_num);
	char write_tx_type(unsigned char id, unsigned char pdo_num, unsigned int new_tx_type);
	char read_inhibition_time(unsigned char id, unsigned char pdo_num);
	char write_inhibition_time(unsigned char id, unsigned char pdo_num, unsigned int new_inhibition_time);
	char read_event_timer(unsigned char id, unsigned char pdo_num);
	char write_event_timer(unsigned char id, unsigned char pdo_num, unsigned int new_event_timer);
	
	char message_rx(unsigned char *buffer,unsigned char *rtr, unsigned char *length, unsigned long *id);
	char ecu_req(unsigned char pid,  char *buffer);
private:
	
};
extern CanbusClass Canbus;
//extern tCAN message;

#endif
