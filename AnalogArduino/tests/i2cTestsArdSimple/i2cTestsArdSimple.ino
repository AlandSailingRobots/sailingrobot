#include <Wire.h>

#define startbyte 0x0F                                 // for i2c communications each datapacket must start with this byte

#define SLAVE_ADDRESS 0x07
byte I2Caddress = SLAVE_ADDRESS;                                       // I2C slave address
int number = 0;
int state = 0;
byte datapacket[20];
byte datapacket2[20];
byte receiveBuf[34];
int dataLength = 9;
int dataLength2 = 7;
int i = 0;

 
void setup() {
 
  Serial.begin(9600);
  loadData();

  Serial.print("I2C Address: ");Serial.println(I2Caddress);
  // initialize i2c as slave
  Wire.begin(I2Caddress);
  
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
  Serial.println("Writing data...");
  Wire.write(datapacket,dataLength+1);
}

void loadData(){
  datapacket[0]=dataLength;
  datapacket[1]=1;
  datapacket[2]=2;
  datapacket[3]=3;
  datapacket[4]=4;
  datapacket[5]=5;
  datapacket[6]=6;
  datapacket[7]=7;
  datapacket[8]=8;
  datapacket[9]=I2Caddress;

  
  datapacket2[0]=dataLength2;
  datapacket2[1]=12;
  datapacket2[2]=22;
  datapacket2[3]=32;
  datapacket2[4]=42;
  datapacket2[5]=52;
  datapacket2[6]=62;
  datapacket2[7]=72;
}

