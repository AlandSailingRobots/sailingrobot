#include <Wire.h>                                      // interrupt based I2C library
#include <EEPROM.h>                                    // library to access EEPROM memory

#define startbyte 0x0F                                 // for i2c communications each datapacket must start with this byte

byte I2Caddress;                                       // I2C slave address
byte datapacket[34];
byte errorflag;
bool request = true;
bool end_request=false; 
byte b;                                                                      // byte from buffer


void setup() {
  Serial.begin(9600);
  TWBR=72;                                           //i2c 100khz 72 100Khz 12 400Khz
  byte i=0;EEPROM.read(0);
  if(i==0x55)                                        // B01010101 is written to the first byte of EEPROM memory to indicate that an I2C address has been previously stored
  {
    I2Caddress=EEPROM.read(1);                       // read I²C address from EEPROM
    Serial.println(I2Caddress, DEC);
  }
  else                                               // EEPROM has not previously been used by this program
  {
    EEPROM.write(0,0x55);                            // set first byte to 0x55 to indicate EEPROM is now being used by this program
    EEPROM.write(1,0x07);                            // store default I²C address
    I2Caddress=0x07;                                 // set I²C address to default
  }
  Wire.begin(I2Caddress);                            // join I²C bus as a slave at I2Caddress
  Wire.onReceive(I2Ccommand);                        // specify ISR for data received
  Wire.onRequest(I2Cstatus);                         // specify ISR for data to be sent
  Serial.println("Finish setup I2c");
  datapacket[0] =0x0B;
}
 
void loop() {
  delay(100);
}

void I2Ccommand(int recvflag){
  
     do                                                                           // check for start byte
     {
       b=Wire.read();                                                             // read a byte from the buffer
       if(b!=startbyte || recvflag!=1)errorflag = errorflag | 1;                 // if byte does not equal startbyte or Master request incorrect number of bytes then generate error
     } while (errorflag>0 && Wire.available()>0);                               // if errorflag>0 then empty buffer of corrupt data
}



void I2Cstatus(){
  datapacket[1]=1;
  datapacket[2]=2;
  datapacket[3]=3;
  datapacket[4]=4;
  datapacket[5]=5;
  datapacket[6]=6;
  datapacket[7]=7;
  datapacket[8]=8;
  datapacket[9]=I2Caddress;
  Wire.write(datapacket,11);
}

