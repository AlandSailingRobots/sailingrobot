/* This example shows how to control a single servo on a Maestro
servo controller using the PololuMaestro library. It assumes you
have an RC hobby servo connected on channel 0 of your Maestro,
and that you have already used the Maestro Control Center
software to verify that the servo is powered correctly and moves
when you command it to from the Maestro Control Center.

Before using this example, you should go to the Serial Settings
tab in the Maestro Control Center and apply these settings:

* Serial mode: UART, fixed baud rate
* Baud rate: 9600
* CRC disabled

Be sure to click "Apply Settings" after making any changes.

This example also assumes you have connected your Arduino to your
Maestro appropriately. If you have not done so, please see
https://github.com/pololu/maestro-arduino for more details on how
to make the connection between your Arduino and your Maestro. */

#include <PololuMaestro.h>

/* On boards with a hardware serial port available for use, use
that port to communicate with the Maestro. For other boards,
create a SoftwareSerial object using pin 10 to receive (RX) and
pin 11 to transmit (TX). */
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(19, 18); //10, 11 for UNO ||19, 18 for MEGA
#endif

/* Next, create a Maestro object using the serial port.

Uncomment one of MicroMaestro or MiniMaestro below depending
on which one you have. */
MicroMaestro maestro(maestroSerial);
//MiniMaestro maestro(maestroSerial);

void setup()
{
  // Set the serial baud rate.
  maestroSerial.begin(9600);
  Serial.begin(9600);
  Serial.println("SETUP COMPLETE");
}

void loop()
{
  /* setTarget takes the channel number you want to control, and
     the target position in units of 1/4 microseconds. A typical
     RC hobby servo responds to pulses between 1 ms (4000) and 2
     ms (8000). */

  // Set the target of channel 3 to 1500 us, and wait 2 seconds.
  maestro.setTarget(4, 5520);
  delay(5000);
  Serial.println("MIDDLE");
    for (uint8_t i = 0; i < 2; i++)
  {
    uint16_t position = maestro.getPosition(i);
    Serial.print("Channel: ");
    Serial.print(i);
    Serial.print(" Position: ");
    Serial.println(position);
  }
  Serial.println();


  // Set the target of channel 4 to 1750 us, and wait 2 seconds.
  maestro.setTarget(4, 5520+1000);
  delay(5000);
  Serial.println("PLUS");
    for (uint8_t i = 0; i < 2; i++)
  {
    uint16_t position = maestro.getPosition(i);
    Serial.print("Channel: ");
    Serial.print(i);
    Serial.print(" Position: ");
    Serial.println(position);
  }
  Serial.println();


  // Set the target of channel 3 to 1250 us, and wait 2 seconds.
  maestro.setTarget(4, 5520-1000);
  delay(5000);
  Serial.println("MINUS");
  
  /* Read the position on the first 2 channels and output it to
  the USB serial connection for the Serial Monitor to see. */
  for (uint8_t i = 0; i < 2; i++)
  {
    uint16_t position = maestro.getPosition(i);
    Serial.print("Channel: ");
    Serial.print(i);
    Serial.print(" Position: ");
    Serial.println(position);
  }

  Serial.println();
  

}
