
/****************************************************************************
ECU CAN-Bus Reader and Logger

Toni Klopfenstein @ SparkFun Electronics
September 2015
https://github.com/sparkfun/CAN-Bus_Shield

This example sketch works with the CAN-Bus shield from SparkFun Electronics.

It enables reading of the MCP2515 CAN controller and MCP2551 CAN-Bus driver.
This sketch also enables logging of GPS data, and output to a serial-enabled LCD screen.
All data is logged to the uSD card. 

Resources:
Additional libraries to install for functionality of sketch.
-SD library by William Greiman. https://github.com/greiman/SdFat 

Development environment specifics:
Developed for Arduino 1.65

Based off of original example ecu_reader_logger by:
Sukkin Pang
SK Pang Electronics www.skpang.co.uk

This code is beerware; if you see me (or any other SparkFun employee) at the local, 
and you've found our code helpful, please buy us a round!

For the official license, please check out the license file included with the library.

Distributed as-is; no warranty is given.
*************************************************************************/

//Include necessary libraries for compilation
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Canbus.h>

//Declare CAN variables for communication
char *EngineRPM;
unsigned char UserInput;
unsigned char rate_num, new_rate;
unsigned char exitChar='x';
unsigned char node_id = 5;
unsigned char action, onOff, new_id, info, r_w, pdo_num,id,ccd,sub;
unsigned char start = 0x01;
unsigned char stop_node = 0x02;
unsigned int input,index;
unsigned long data;

char buffer[64];  //Data will be temporarily stored to this buffer before being written to the file

//********************************Setup Loop*********************************//
void setup() {
  //Initialize Serial communication for debugging
  Serial.begin(9600);
  Serial.println("my ECU Sender");
  
  //Initialize CAN Controller 
  if(Canbus.init(CANSPEED_250))  /* Initialize MCP2515 CAN controller at the specified speed */
  {
    Serial.println("CAN Init Ok");
    delay(1500);
  } 
  else
  {
    Serial.println("Can't init CAN");
    return;
  } 
  
  Serial.println("Press any key to begin");
  
  wait();
  UserInput = Serial.read();
  Serial.println("Enter message:");
  Canbus.message_NMT(node_id,start);

  Serial.println("Choose Function");
  Serial.println("n = NMT, l = LSS, p = PDO, s = SDO, t = TX, e = English Options");
}

//********************************Main Loop*********************************//
void loop(){

  while(Serial.available()){
    UserInput = Serial.read();
    
    switch(UserInput) {
      case 'n':
        Serial.println("Enter target node:");
        wait();
        node_id=Serial.read()-'0';
        node_id=5;
        Serial.println("Enter action:");
        Serial.println("s = start, x = stop, p = preoperational, r = reset, c = reset communication");
        wait();
        action = Serial.read();
        switch(action) {
          case 's':
            Canbus.start_node(node_id);
            break;
          case 'x':
            Canbus.stop_node(node_id);
            break;
          case 'p':
            Canbus.preop_node(node_id);
            break;
          case 'r':
            Canbus.reset_node(node_id);
            break;
          case 'c':
            Canbus.reset_comm_node(node_id);
            break;
        }
        break;

        case 'l':
          Serial.println("Enter action:");
          Serial.println("c = Switch Config Mode, i = Change ID, b = Change Baud Rate, s = Save, g = Get Info, n = Identify Nonconfigured Nodes");
          wait();
          action = Serial.read();
          switch(action) {
            case 'c':
              Serial.println("ON or OFF:");
              Serial.println("1 = ON, 0 = OFF");
              wait();
              onOff = Serial.read()-'0';
              Canbus.switch_lss_config(onOff);
              break;
            case 'i':             
              Serial.println("New ID:");
              wait();
              new_id = Serial.read()-'0';
              Canbus.change_id(new_id);
              break;
            case 'b':
              Serial.println("New rate?");
              Serial.println("1=125, 2=250");
              wait();
              rate_num= Serial.read();
              switch(rate_num) {
                case '1':
                  new_rate = LSS_BAUD_125;
                  break;
                case '2':
                  new_rate = LSS_BAUD_250;
                  break;
                default:
                  Serial.println("Incorrect Input");
                  break;
              }
              Canbus.change_baud_rate(new_rate);
              break;
            case 's':
              Canbus.save_lss_config();
              break;
            case 'g':
              Serial.println("Enter desire info:");
              Serial.println("n = node ID, s = serial number, r = revision number, p = product code, v = vendor id");
              wait();
              info = Serial.read();
              switch(info) {
                case 'n':
                  Canbus.get_node_id();
                  break;
                case 's':
                  Canbus.get_serial_num();
                  break;
                case 'r':
                  Canbus.get_revision_num();
                  break;
                case 'p':
                  Canbus.get_product_code();
                  break;
                case 'v':
                  Canbus.get_vendor_id();
                  break;
              }
              break;
            case 'n':
              Canbus.identify_non_configured();
              break;
            }
        break;
            

        case 's':
          Serial.println("Node ID (read[r]/write[w], pdo number):");
          wait();
          node_id = Serial.read()-'0';
          Serial.println("Read[r]/write (pdo number):");
          wait();
          r_w = Serial.read();
          Serial.println("Pdo number:");
          wait();
          pdo_num = Serial.read()-'0';
          Serial.println("Component:");
          Serial.println("h = heartbeat, p = pos, n = number of entries, c = COB, t = TX type, i = inhibition time, e = event timer");
          wait();
          action = Serial.read();
          switch(action) {
            case 'h':
              Canbus.read_heartbeat(node_id);
              break;
            case 'p':
              Canbus.read_pos(node_id);
              break;
            case 'n':
              Canbus.read_num(node_id,pdo_num);
              break;
            case 'c':
              Canbus.read_cob(node_id,pdo_num);
              break;
            case 't':
              Canbus.read_tx_type(node_id,pdo_num);
              break;
            case 'i':
              Canbus.read_inhibition_time(node_id,pdo_num);
              break;
            case 'e':
              if (r_w=='w') {
                  Serial.println("Enter new event timer value");
                  data=pars_serial();
                  Canbus.write_event_timer(node_id,pdo_num,data);             
                }
              Canbus.read_event_timer(node_id,pdo_num);
              break;
         }

        break;

        case 'd':
          id=pars_serial();
          ccd=pars_serial();
          index=pars_serial();
          sub=pars_serial();
          data=pars_serial();
          Canbus.message_SDO(id,ccd,index,sub,data);
          break;
    }
    
    Serial.println("Choose Function");
    Serial.println("n = NMT, l = LSS, p = PDO, s = SDO, t = TX, e = English Options");
  }
}

void wait(){
    while(!Serial.available()){
    //Loop until input received
  }
}

unsigned long pars_serial(){
  unsigned long ret=0;
  int count = 0;
  unsigned char in=0;
  wait();
  in = Serial.read()-'0';
  while(in!=('e'-'0')){
    if((in>16)&&(in<23)){
      in=in-7;
    }
    ret = in + (ret << 4);
    count++;
    wait();
    in = Serial.read()-'0';
  }
  Serial.println(ret,HEX);
  return ret;
}

