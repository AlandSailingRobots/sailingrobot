#include <Wire.h>
#include <EEPROM.h>                                    // library to access EEPROM memory

#define startbyte 0x0F                                 // for i2c communications each datapacket must start with this byte

#define SLAVE_ADDRESS 0x07
byte I2Caddress;                                       // I2C slave address
int number = 0;
int state = 0;
byte datapacket[34];
byte receiveBuf[34];
int dataLength = 12;
int i = 0;

 
void setup() {
 
  Serial.begin(9600);
  loadData();
  
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

  Serial.print("I2C Address: ");Serial.println(I2Caddress);
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}
 
void loop() {
 delay(100);
}
 
// callback for received data
void receiveData(int byteCount){
  Serial.println("Receiving Data...");
  i=0;
  
  while(Wire.available()) {
  number = Wire.read();
  receiveBuf[i]=number;
  i++;
  Serial.print(number); Serial.print(" | ");
  }
  Serial.print("Read "); Serial.print(i); Serial.println("bytes.");
}
 
// callback for sending data
void sendData(){
 Wire.write(datapacket,dataLength);
}

void loadData(){
  datapacket[1]=1;
  datapacket[2]=2;
  datapacket[3]=3;
  datapacket[4]=4;
  datapacket[5]=5;
  datapacket[6]=6;
  datapacket[7]=7;
  datapacket[8]=8;
  datapacket[9]=I2Caddress;
}

