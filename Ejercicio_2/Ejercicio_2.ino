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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define IS_MASTER true           // Set Master mode for controlling device, for sensor device set to false

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Connect OLED cables to the following port
// red    -> 3.3V/5V
// black  -> GND
// yelow  -> scl
// blue   -> sda

// Protocol
/*
- help                                              -> Show available commands
- us <srf02> {one-shot | on <period_ms> | off}      -> Start a single measurement, a continous measurment for <period_ms> duration, stop periodic measurments. <srf02> refers to the sensors address
- us <srf02> unit {inc | cm | ms}                   -> Set the unit for the measurments < inc | cm | ms >
- us <srf02> delay <ms>                             -> Set delay between cosecutive measurments           
- us <srf02> status                                 -> Return configuation of sensor (address, state, units, delay)
- us                                                -> Return all available sensors (addresses)
*/

// Define codes for 1st byte
#define CODE_US 0b0000001
#define CODE_UNIT 0b00000010
#define CODE_STATUS 0b00000011
#define CODE_RESP 0b11000000
#define CODE_DELAY 0b00000100
#define CODE_OFF 0b01000011

// Define codes for payload
#define CODE_ONE_SHOT 0b0100001
#define CODE_ON_PERIOD_MS 0b0100010
#define CODE_UNIT_INC 0b01010000
#define CODE_UNIT_CMS 0b01010001
#define CODE_UNIT_MS 0b01010010

// Defeine code for ACK/NACK
#define CODE_ACK 0b10000000
#define CODE_NACK 0b10000001

#define NUM_CHARS 32



constexpr const uint32_t serial_monitor_bauds=115200;
constexpr const uint32_t serial1_bauds=9600;

//constexpr const uint32_t pseudo_period_ms=1000;

uint32_t last_ms;
uint32_t new_ms;


uint8_t header = 0;                       // indicate which commade is to be sent
bool serialNewData = false;

char serial1ReceivedChars[NUM_CHARS];     // an array to store the received command


void setup() {

  // Setup indicator LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial port for communication between microcontrollers
  Serial1.begin(serial1_bauds);

  // Initialize serial monitor 
  Serial.begin(serial_monitor_bauds);

  // Blink LED if not connected to Serial
  int count = 0;
  while (!Serial && count < 25){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    count++;
  }

  // Inicialización del puerto de comunicaciones con el otro dispositivo MKR 
  Serial1.begin(serial1_bauds);

#if IS_MASTER

  masterSetup();

#else

  slaveSetup();

#endif
}

void loop() {

#if IS_MASTER

  masterLoop();

#else

  slaveLoop();

#endif

}




