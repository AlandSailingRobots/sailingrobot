// demo: CAN-BUS Shield, receive data with interrupt mode
// when in interrupt mode, the data coming can't be too fast, must >20ms, or else you can use check mode
// loovee, 2014-6-13

#include <SPI.h>
#include <mcp_can.h>
#include <AHM36A.h>
#include <NMEA2000_ext.h>

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 53; //uno = 10, mega = 53

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned long id = 0;
unsigned char buf[8];
char str[20];
unsigned char encoder_id = 5;

bool record = false;

void setup()
{
    Serial.begin(9600);


START_INIT:
    
    if(CAN_OK == CAN.begin(CAN_250KBPS,1))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init ok!");
    }
    else
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }
    
    attachInterrupt(digitalPinToInterrupt(2), MCP2515_ISR, FALLING); // start interrupt
}

void MCP2515_ISR()
{
    flagRecv = 1;
}

void loop()
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

            if (record) 
            {
              Serial.print("#S|S|[");  
            }
            
            
            Serial.print(id,HEX);
            Serial.print(", ");
            Serial.print(len,HEX);
            for(int i = 0; i<len; i++)
            {
              Serial.print(", ");
              Serial.print(buf[i],HEX);
            }
            if (record) 
            {
              Serial.print("]#");
            }
            Serial.println("");
        }
    }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
