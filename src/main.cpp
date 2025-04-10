/*--------------------------------------------------------------------
Name:   Luke Doughty
Date:   3/3/2025
File:   main.cpp

Purp: Taking readings from the ICM-20948 IMU sensor using SPI and storing 
them to microSD on a second arduino, using UART.

Wiring: My setup has a Teensy 4.1 as the master and a Teensy 3.5 as the slave with the MicroSD slot.
To connect the UART from the master Teensy 4.1 to the slave Teensy 3.5:
  Connect the master's RX5 (pin 21) to the slave's TX1 (pin 33). (ORANGE WIRE)
  Connect the master's TX5 (pin 20) to the slave's RX1 (pin 32). (YELLOW WIRE)
  Connect the master's 5V to the slave's 5V. (RED WIRE)
  Connect the master's GND to the slave's GND. (GREEN WIRE)
To connect the master Teensy4.1 to the Sparkfun Qwiic 9Dof IMU ICM-20948:
  Conn


Derived from SparkFun 9DoF IMU Breakout - ICM 20948 Example999_Portable.ino
by Owen Lyke @ SparkFun Electronics, April 17 2019
--------------------------------------------------------------------*/
// #define SECOND_TEENSY
#include <Arduino.h>

#define SECOND_TEENSY
#ifdef SECOND_TEENSY // ---------------------------------------------
#include <SD.h>

#define BUFFER_SIZE 64
#define FILENAME "teensySD.txt"

File secondTeensySD;

void blink(int time){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(time);
  digitalWrite(LED_BUILTIN, LOW);
  delay(time);
}

void setup(){
  Serial.begin(9600); // Serial Monitor
  Serial1.begin(9600); // UART from primary Teensy
  pinMode(LED_BUILTIN, OUTPUT);
  while (!SD.begin(BUILTIN_SDCARD)){
    // Can't connect to the SD card.
    blink(250);
  };
  while (!Serial && !Serial1) {
    // Cant connect to the serial monitor or the other Teensy.
    blink(100);
  };
  secondTeensySD = SD.open(FILENAME, FILE_WRITE);
  if (secondTeensySD) {
    secondTeensySD.print("Connected to secondTeensySD at epoch: ");
    secondTeensySD.println(millis());
    secondTeensySD.flush();
    digitalWrite(LED_BUILTIN, HIGH);

  }
}


void bufferAndSaveToSD(char receivedByte) {
  static char serialBuffer[BUFFER_SIZE];
  static unsigned int bufferIndex = 0;
  serialBuffer[bufferIndex] = receivedByte;
  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE - 1 || receivedByte == '\n' || receivedByte == '\r') {
    // if the buffer is full or newline, write to SD and reset the index.
    serialBuffer[bufferIndex] = '\0'; // Null-terminate the string
    secondTeensySD.write(serialBuffer);
    secondTeensySD.flush();
    bufferIndex = 0;
  }
}

void loop(){
  digitalWrite(LED_BUILTIN, LOW);
  if (!secondTeensySD) {
    // Can't open the SD card.
    // Doesnt respond to removing the SD card ):
    blink(250);
    return;
  }
  if(!Serial1 && !Serial){
    // Not receiving any data (Neither from the other Teensy nor Serial Monitor).
    blink(100);
    secondTeensySD.println("Serial connection failed.");
    secondTeensySD.flush();
    return;
  };
  if (Serial1.available() > 0 && secondTeensySD) {
    char receivedByte = Serial1.read();
    bufferAndSaveToSD(receivedByte);
  }
  else if (Serial.available() > 0 && secondTeensySD) {
    char receivedByte = Serial.read();
    bufferAndSaveToSD(receivedByte);
  }
}
#else // -----------------------------------------------------------
#include "ICM_20948.h"
#include "imu.h"

ICM_20948_Device_t myICM;

void setup()
{
  Serial1.begin(115200); // UART to other Teensy
  pinMode(LED_BUILTIN, OUTPUT);
  setupIMU(&myICM);
}

void loop()
{
  delay(1000);

  ICM_20948_AGMT_t agmt = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
  if (ICM_20948_get_agmt(&myICM, &agmt) == ICM_20948_Stat_Ok)
  {
    printRawAGMT(agmt);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Uh oh");
  }
}
#endif // -----------------------------------------------------------
