#include <Wire.h>
#include "DataTransferIDs.h"

#include <SPI.h>
#include <mcp_can.h>

#define INIT_ID 0xFF                                 // for i2c communications each datapacket must start with this byte
#define SLAVE_ADDRESS 0x07


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned long id = 0;
unsigned char buf[8];
char str[20];

byte I2Caddress = SLAVE_ADDRESS;                                       // I2C slave address
bool initialized;

int dataLength;
byte dataID;
byte datapacket[32];
byte receiveBuf[34];
byte CANID;

byte actuatorFeedback[4];
byte windSensorFeedback[4];
byte analogFeedback[4];
bool actuatorFeedbackReady;
bool windSensorReady;
bool analogReady;

int readVal = 0;
int i,j;

int pressurePin = A0;
int batteryPin = A1;
 
void setup() {
 
  Serial.begin(9600);
  
  START_INIT:

    if(CAN_OK == CAN.begin(CAN_500KBPS,1))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        //goto START_INIT;
    }

  attachInterrupt(digitalPinToInterrupt(2), MCP2515_ISR, FALLING); // start interrupt

  pinMode(pressurePin,INPUT);
  pinMode(batteryPin,INPUT);
  
  Serial.print("I2C Address: ");Serial.println(I2Caddress);
  // initialize i2c as slave
  Wire.begin(I2Caddress);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  //Set init_flag false
  initialized = false;
}
 
void loop() {
  readCAN();
  readAnalogData();
  delay(100);
}
 
// callback for received data
void receiveData(int byteCount){
  
  Serial.println("Receiving Data...");
  i=0;

  if (Wire.available())
  {
    readVal = Wire.read();
    CANID = readVal;    
    Serial.print("CANID: "); Serial.print(readVal); Serial.print(" || ");
  }
  
  while(Wire.available()) 
  {
    readVal = Wire.read();
    receiveBuf[i]=readVal;
    i++;
    Serial.print(readVal); Serial.print(" | ");
  }
  Serial.print("Read "); Serial.print(i); Serial.println(" bytes.");  
  CAN.sendMsgBuf(CANID, 0, (byte) DataIDToByteLength(CANID), receiveBuf);
  Serial.print("Sent "); Serial.print(DataIDToByteLength(CANID)); Serial.println(" CAN bytes");
}
 
// callback for sending data
void sendData(){

  if (!initialized)
  {
    dataLength = 1;
    datapacket[0]=dataLength;
    datapacket[1]= INIT_ID;
    Serial.println("Writing init data...");
    Wire.write(datapacket, dataLength+1);

    dataLength = 0;
  }

  else
  {
    loadData();
    Serial.println("Writing data...");
    Wire.write(datapacket, dataLength+1);
    clearData();
  }
}

void readAnalogData()
{
  int pressure = analogRead(pressurePin);
  int battery = analogRead(batteryPin);
  analogFeedback[0] = pressure << 8;
  analogFeedback[1] = pressure & 0x00FF;
  analogFeedback[2] = battery << 8;
  analogFeedback[3] = battery & 0x00FF;

  analogReady=true;
}

void clearData()
{
  actuatorFeedbackReady = false;
  windSensorReady = false;
  analogReady = false;

  dataLength = 0;
  dataID = 0;
}

void loadData(){
  dataID = 0;
  dataLength = 1;
  i = dataLength+1;

  if (actuatorFeedbackReady)
  {
    j = 0;
    dataLength += DataIDToByteLength(DataID::ActuatorFeedback); 

    while (i < dataLength+1)
    {
      datapacket[i]=actuatorFeedback[j];
      i++;
      j++;
    }
    
    dataID |= DataID::ActuatorFeedback;
    
  }
 
  if (windSensorReady)
  {
    j = 0;
    dataLength += DataIDToByteLength(DataID::WindSensorFeedback); 

    while (i < dataLength+1)
    {
      datapacket[i]=windSensorFeedback[j];
      i++;
      j++;
    }
    
    dataID |= DataID::ActuatorFeedback;
    
  }
  
  if (analogReady)
  {
    j = 0;
    dataLength += DataIDToByteLength(DataID::AnalogFeedback); 

    while (i < dataLength+1)
    {
      datapacket[i]=analogFeedback[j];
      i++;
      j++;
    }
    
    dataID |= DataID::AnalogFeedback;
    
  }
}

void MCP2515_ISR()
{
    flagRecv = 1;
}

void readCAN()
{
  if(flagRecv) 
  {                                   // check if get data
  
      flagRecv = 0;                   // clear flag
  
      // iterate over all pending messages
      // If either the bus is saturated or the MCU is busy,
      // both RX buffers may be in use and reading a single
      // message does not clear the IRQ conditon.
      while (CAN_MSGAVAIL == CAN.checkReceive()) 
      {
          // read data,  len: data length, buf: data buf
          CAN.readMsgBufID(&id, &len, buf);
  
          Serial.print("ID: ");
          Serial.print(id,HEX);
          Serial.print(" || Message: ");
          // print the data
          for(int i = 0; i<len; i++)
          {
              Serial.print(buf[i]);Serial.print(" | ");
          }
          Serial.println();
      }
  }
}

