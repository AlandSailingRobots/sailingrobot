#include <Wire.h>                                      // interrupt based I2C library
#include <EEPROM.h>                                    // library to access EEPROM memory
#define startbyte 0x0F                                 // for i2c communications each datapacket must start with this byte
#define PRESSUREBYTE 0x01                              // request byte for pressure data
#define ADDRESSBYTE 0x22                               // request byte for i2c address store in eeprom
#define RUDDERBYTE 0x02                               // request byte for rudder data
#define SHEETBYTE 0x03                               // request byte for sheet data
#define BATTERYBYTE 0x04                               // request byte  for battery data
#define ERRORBYTE 0x00                                 // error byte if byte receiving wrong
 
int pressurePin = A0;
int rudderPin = A1;
int sheetPin = A2;
int batteryPin = A3;
 
byte mask = 1;
byte LSB[4] = {0,0,0,0};
byte MSB[4] = {0,0,0,0};
 
byte i2cfreq;                                          // I2C clock frequency can be 100kHz(default) or 400kHz
byte I2Caddress;                                       // I2C slave address
byte datapacket[2];
byte errorflag;
bool request_activated = false;
bool request = true;
bool end_request=false;
byte b;                                                                      // byte from buffer
 
 
void setup() {
  Serial.begin(9600);
  TWBR=72;                                           //i2c 100khz 72 100Khz 12 400Khz
  pinMode(pressurePin, INPUT);
  pinMode(rudderPin, INPUT);
  pinMode(sheetPin, INPUT);
  pinMode(batteryPin, INPUT);
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
}
 
void loop() {
  splitValue(analogRead(pressurePin),PRESSUREBYTE);
  splitValue(analogRead(rudderPin),RUDDERBYTE);
  splitValue(analogRead(sheetPin),SHEETBYTE);
  splitValue(analogRead(batteryPin),BATTERYBYTE);
  delay(100);
}
 
void splitValue(int inputValue,byte sensor) {
  LSB[sensor-1] = inputValue & 0xFF;
  MSB[sensor-1] = inputValue >> 8;
  Serial.print("Device : ");Serial.print(sensor,HEX);
  Serial.print("inputValue: "); Serial.print(inputValue, BIN);
  Serial.print("  MSB: "); Serial.print(MSB[sensor-1], BIN);
  Serial.print("  LSB: "); Serial.print(LSB[sensor-1], BIN);
  Serial.print("  value: "); Serial.println(inputValue);
}
 
void I2Ccommand(int recvflag){
 
  int i;                                                                       // integer from buffer
  if (!request_activated)
  {
     do                                                                           // check for start byte
     {
       b=Wire.read();                                                             // read a byte from the buffer
       if(b!=startbyte || recvflag!=1)errorflag = errorflag | 1;                 // if byte does not equal startbyte or Master request incorrect number of bytes then generate error
     } while (errorflag>0 && Wire.available()>0);                               // if errorflag>0 then empty buffer of corrupt data
     if (b==startbyte)
        request_activated = true;
  }
  else{
    b = Wire.read();
    if (b == PRESSUREBYTE || b == ADDRESSBYTE || b == RUDDERBYTE || b == SHEETBYTE || b == BATTERYBYTE){                         //
      request = true;
      end_request=false;
    }
    else{                                                               // wrong byte received flushing i2c buffer
       do                                                                           // check for start byte
       {
         Wire.read();                                                             // read a byte from the buffer
       } while (Wire.available()>0);
       b=ERRORBYTE;
    }
    request_activated=false;
  }
}
 
 
 
void I2Cstatus(){
  if (!end_request &&(b == PRESSUREBYTE || b == RUDDERBYTE || b == SHEETBYTE || b == BATTERYBYTE)){
    if (request){
      datapacket[0] = MSB[b-1];
      request = false;
    }
    else{
      datapacket[0] =LSB[b-1];
      end_request = true;
    }
  }
 
  else if (!end_request && b==ADDRESSBYTE){
     datapacket[0] =I2Caddress;
     end_request = true;
     request=false;
  }
  else
    datapacket[0] = ERRORBYTE;
  Wire.write(datapacket,1);
}