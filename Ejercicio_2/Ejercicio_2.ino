/*
* This code uses snippets and functions derived from "Internet de las Cosas" at ULPGC
* Code that was not written by myself is marked by a comment "adapted" in the function desciption
* 
* This code is part of the first assignment for the course on "Internet de las Cosas" at ULPGC
*
* Author: Karl Deilmann
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define IS_MASTER true    // Set Master mode for controlling device, for sensor device set to false


constexpr const uint32_t serial_monitor_bauds=115200;
constexpr const uint32_t serial1_bauds=9600;

constexpr const uint32_t pseudo_period_ms=1000;

const byte numChars = 32;


// Code for Master
#if IS_MASTER

  char serialReceivedChars[numChars];   // an array to store the received command

  uint8_t opbyte=0;

  boolean serialNewData = false;

// Code for Slave
#else

  uint8_t led_state=LOW;

  char serial1ReceivedChars[numChars];   // an array to store the received command

  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  boolean serial1NewData = false;


#endif

// yelow scl
// blue sda

void setup() {

  // Setup indicator LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup serial connection
  // SerialUSB.begin(115200);

  // Inicialización del puerto para el serial monitor 
  Serial.begin(serial_monitor_bauds);

  while (!Serial);

  // Inicialización del puerto de comunicaciones con el otro dispositivo MKR 
  Serial1.begin(serial1_bauds);

  #if IS_MASTER

    Serial.println("Use 'help' to view available commands");

  #else

    Serial.println("waiting...");   

  #endif
}

void loop() {

  #if IS_MASTER

    inputData();          // Checks if there is any new data to be read from the serial monitor
    showData();           // Shows the new data if there is any
    checkData();          // checks for any syntax error or 'help' command
    sendDataToSlave();    // Send the data to the Slave

  #else

    checkSerial1();     // Checks if there is any new data to be read from the serial1
    showData();         // Shows the new data if there is any

  #endif

}


#if IS_MASTER

  void inputData() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && serialNewData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            serialReceivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            serialReceivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            serialNewData = true;

        }
    }
  }

  void showData() {
    if (serialNewData == true) {
      Serial.println(serialReceivedChars);
      Serial.println();
    }
  }

  void checkData(){
    if (strcmp(serialReceivedChars, "help") == 0 && (serialNewData == true)){
      Serial.println("us <srf02> {one-shot | on <period_ms> | off}");
      Serial.println("us <srf02> unit {inc | cm | ms}");
      Serial.println("us <srf02> delay <ms>");
      Serial.println("us <srf02> status");
      Serial.println("us");
      Serial.println();
      serialNewData = false;
    }
  }

  void sendDataToSlave() {

    if (serialNewData == true) {
      Serial.println("sending...");

      
      Serial1.write(serialReceivedChars);
      

      serialNewData = false;
    }
  }

#else

  void checkSerial1(){

  while (Serial1.available() > 0 && serial1NewData == false){
    rc = Serial1.read();

        if (rc != endMarker) {
            serial1ReceivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            serial1ReceivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            serial1NewData = true;

        }
    }
  }

  void showData() {
    if (serial1NewData == true) {
      Serial.println(serial1ReceivedChars);
      Serial.println();
      serial1NewData = false;
    }
  }

  void acknowledge(){
    if (serial1NewData == true) {
      Serial1.write(1);
      serial1NewData = false;
    }
  }

#endif


