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

///////////////////////////////
/// tc_lib special settings ///
///////////////////////////////

//#define ARDUINO_MKR_PORT
//
//#include "tc_lib.h"
//
//using namespace tc_lib::arduino_mkr;
//
//#define CALLBACK_PERIOD 50000000 // hundreths of usecs. (1e-8 secs.)
//#define DELAY_TIME 1000 // msecs.
//
//// action_tc4 declaration
//action_tc4_declaration();
//
//struct ctx
//{
//  ctx() { counter=0; }
//
//  volatile uint32_t counter;
//  timer_t start = millis();
//};
//
//ctx action_ctx;

///////////////////////////////
///////////////////////////////
///////////////////////////////



#define IS_MASTER false    // Set Master mode for controlling device, for sensor device set to false


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

// Define codes for 3rd byte
#define CODE_ONE_SHOT 0b0100001
#define CODE_ON_PERIOD_MS 0b0100010
#define CODE_OFF 0b01000011
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



bool serial1NewData = false;

char serial1ReceivedChars[NUM_CHARS];   // an array to store the received command


// Code for Master
#if IS_MASTER

  uint8_t header;             // indicate which commade is to be sent
  uint8_t payload[4] = {};
  bool temp = false;

  uint8_t acknowledge;

  uint8_t result;
  char *output;

// Code for Slave
#else

  ////////////////////
  /// OLED Display ///
  ////////////////////

  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels

  time_t lastTimeOLED_ms = 0;           // Variable to hold a time stamp in milli seconds to time OLED refreshing
  #define OLED_REFRESH_RATE_MS 100     // Define refreshrate of OLED in ms

  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  // The pins for I2C are defined by the Wire-library. 
  // On an arduino UNO:       A4(SDA), A5(SCL)
  // On an arduino MEGA 2560: 20(SDA), 21(SCL)
  // On an arduino LEONARDO:   2(SDA),  3(SCL), ...
  #define OLED_RESET     -1     // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3D   // /< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



  //////////////////////
  /// Sensor reading ///
  //////////////////////

  time_t lastTimeSendor_ms = 0;      // Variable to hold a time stamp in milli seconds to time sensor refreshing

  #define SRF02_I2C_ADDRESS byte((0xE0)>>1)
  #define SRF02_I2C_INIT_DELAY 100    // in milliseconds
  #define SRF02_RANGING_DELAY 70      // milliseconds
  #define SRF02_MEASURE_DELAY 500    // milliseconds

  // LCD05's command related definitions        // TODO: Remove if not necessary
  #define COMMAND_REGISTER byte(0x00)
  #define SOFTWARE_REVISION byte(0x00)
  #define RANGE_HIGH_BYTE byte(2)
  #define RANGE_LOW_BYTE byte(3)
  #define AUTOTUNE_MINIMUM_HIGH_BYTE byte(4)
  #define AUTOTUNE_MINIMUM_LOW_BYTE byte(5)

  // SRF02's command codes
  #define REAL_RANGING_MODE_INCHES    byte(80)
  #define REAL_RANGING_MODE_CMS       byte(81)
  #define REAL_RANGING_MODE_USECS     byte(82)
  #define FAKE_RANGING_MODE_INCHES    byte(86)
  #define FAKE_RANGING_MODE_CMS       byte(87)
  #define FAKE_RANGING_MODE_USECS     byte(88)
  #define TRANSMIT_8CYCLE_40KHZ_BURST byte(92)
  #define FORCE_AUTOTUNE_RESTART      byte(96)
  #define ADDRESS_CHANGE_1ST_SEQUENCE byte(160)
  #define ADDRESS_CHANGE_3RD_SEQUENCE byte(165)
  #define ADDRESS_CHANGE_2ND_SEQUENCE byte(170)

  uint16_t srf_range = 0;     // Global variable to hold range measurement of ultrasonic sensor
  uint16_t srf_min = 0;       // Global variable to hold min-range measurement of ultrasonic sensor

  // Flag to show that periodic sensor reading is active
  bool _is_periodic_ranging = false;
  time_t sensorFirstTime = 0;
  time_t sensorLastTime = millis();

  inline void write_command(byte address,byte command)
  { 
    Wire.beginTransmission(address);
    Wire.write(COMMAND_REGISTER); 
    Wire.write(command); 
    Wire.endTransmission();
  }

  byte read_register(byte address,byte the_register)
  {
    Wire.beginTransmission(address);
    Wire.write(the_register);
    Wire.endTransmission();
    
    // getting sure the SRF02 is not busy
    Wire.requestFrom(address,byte(1));
    while(!Wire.available()) { }
    return Wire.read();
  }


  /////////////////////
  /// Communication ///
  /////////////////////

  #define DATA_ARR_LEN 3

  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  uint8_t counter = 0;
  uint8_t header = 0;

  bool rec = false;

  uint8_t data[DATA_ARR_LEN] = {};

  // Enum to tell unit apart in srf
  enum SensorUnit{
    INC,
    CMS,
    USEC
  };

  uint16_t sensorAddress = SRF02_I2C_ADDRESS;
  uint16_t sensorPeriod_ms = 10000;                     // Default value for debugging
  uint16_t sensorDelay_ms = SRF02_MEASURE_DELAY;
  uint8_t sensorUnit = CMS;

#endif

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

    Serial.println("Use 'help' to view available commands");

  #else

    // DEBUGGING
    _is_periodic_ranging = true;
    SerialUSB.println("DEBUGGING: Strating period sensor reading once...");

    Serial.println("waiting...");

    ////////////////////
    /// OLED Display ///
    ////////////////////

    Serial.println("Begining OLED setup ...");
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(200); // Pause for 200 milli seconds

    // Clear the buffer
    display.clearDisplay();

    Serial.println("Finished OLED setup ...");

    //////////////////////
    /// Sensor reading ///
    //////////////////////

    Serial.println("Begining Sensor setup ...");
    Serial.println("initializing Wire interface ...");
    Wire.begin();
    delay(SRF02_I2C_INIT_DELAY);  
  
    byte software_revision=read_register(SRF02_I2C_ADDRESS,SOFTWARE_REVISION);
    Serial.print("SFR02 ultrasonic range finder in address 0x");
    Serial.print(SRF02_I2C_ADDRESS,HEX); Serial.print("(0x");
    Serial.print(software_revision,HEX); Serial.println(")");

    Serial.println("Finished Sensor setup ...");

  #endif
}

void loop() {

#if IS_MASTER

  inputData();          // Checks if there is any new data to be read from the serial monitor
  showData();           // Shows the new data if there is any
  checkData();          // checks for any syntax error or 'help' command
  sendDataToSlave();    // Send the data to the Slave

#else

  ////////////////
  /// Protocol ///
  ////////////////

  checkSerial1();     // Checks if there is any new data to be read from the serial1
  //showData();         // Shows the new data if there is any

  ////////////////////
  /// OLED Display ///
  ////////////////////

  
  // Refresh OLED after 100ms
  if (millis() - lastTimeOLED_ms > OLED_REFRESH_RATE_MS) {
    displayDataToOLED(srf_range, srf_min);
    lastTimeOLED_ms = millis();
  }

  //////////////////////
  /// Sensor reading ///
  //////////////////////


  // If a period for sensor readout is demanded, enter here
  if(_is_periodic_ranging) {
    time_t cur_time = millis();
    // Read sensor if delay has passed
    if (cur_time - sensorLastTime > sensorDelay_ms) {
      sensorLastTime = millis();    // Set last time variable to curent time
      com_srfOneShot();             // Execute sensor reading (will send data to master)
      Serial.println("Executed reading");
    }
    // Determine if sensor ranging time has passed
    if (cur_time - sensorFirstTime > sensorPeriod_ms) {
      _is_periodic_ranging = false; // stop periodic sensor reading
    }
  }

  //// Measure after SRF02_MEASURE_DELAY ms
  //if (millis() - lastTimeSendor_ms > SRF02_MEASURE_DELAY && !isActiveRanging) {
  //  write_command(SRF02_I2C_ADDRESS,REAL_RANGING_MODE_CMS);
  //  // Delay to make accurate reading -> see manual
  //  delay(SRF02_RANGING_DELAY);
  //
  //  byte high_byte_range=read_register(SRF02_I2C_ADDRESS,RANGE_HIGH_BYTE);
  //  byte low_byte_range=read_register(SRF02_I2C_ADDRESS,RANGE_LOW_BYTE);
  //  byte high_min=read_register(SRF02_I2C_ADDRESS,AUTOTUNE_MINIMUM_HIGH_BYTE);
  //  byte low_min=read_register(SRF02_I2C_ADDRESS,AUTOTUNE_MINIMUM_LOW_BYTE);
  //
  //  srf_range = int16_t((high_byte_range<<8) | low_byte_range);
  //  srf_min = int16_t((high_min<<8) | low_min);
  //
  //  // Reset timer variable
  //  lastTimeSendor_ms = millis();
  //}

#endif

}


#if IS_MASTER

void checkSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0 && serialNewData == false) {
      rc = Serial.read();

      if (rc != endMarker) {
          serialReceivedChars[ndx] = rc;
          ndx++;
          if (ndx >= NUM_CHARS) {
              ndx = NUM_CHARS - 1;
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

  if (serialNewData == true){

    if (strcmp(serialReceivedChars, "help") == 0) {
      Serial.println("us <srf02> {one-shot | on <period_ms> | off}");
      Serial.println("us <srf02> unit {inc | cm | ms}");
      Serial.println("us <srf02> delay <ms>");
      Serial.println("us <srf02> status");
      Serial.println("us");
      Serial.println();
      serialNewData = false;
      return;
    }

    char substring0[2] = {
      serialReceivedChars[0],
      serialReceivedChars[1]
      };

    if (strcmp(substring0, "us") == 0){

      if (serialReceivedChars[2] == '\0') {
        header = 9;
        payload[0] = 0;
        payload[1] = 0;
        payload[2] = 0;
        payload[3] = 0;

        printHeaderAndData();
        return;
      }

      else{
        // take address and aconvert it to uint8
        char strHexToUint8[3] = {
          serialReceivedChars[5],
          serialReceivedChars[6],
          '\0'
        };

        result = strtol(strHexToUint8, &output, 16);

        // if there is any error we break out of the function
        if (result == 0){ 
          Serial.println("Error");
          serialNewData = false;
          Serial.println("Use 'help' to view available commands");
          return;
        }

        // we store the address
        payload[0] = result;
        
        char substring1[6] = {
          serialReceivedChars[8],
          serialReceivedChars[9],
          serialReceivedChars[10],
          serialReceivedChars[11],
          serialReceivedChars[12],
          '\0'
          };

        if (strcmp(substring1, "one-s\0") == 0){
          
          header = 1;
          payload[1] = 0;
          payload[2] = 0;
          payload[3] = 0;

          printHeaderAndData();
          return;
        }



        char substring2[4] = {
          serialReceivedChars[8],
          serialReceivedChars[9],
          serialReceivedChars[10],
          '\0'
          };

        if (strcmp(substring2, "on \0") == 0){
          
          header = 2;

          uint8_t i=11;
          char substring3[32];
          while (serialReceivedChars[i] != '\0'){
            substring3[i-11] = serialReceivedChars[i];
            i++;
          }

          int int_result = strtol(substring3, &output, 10);

          Serial.print("Stored number: ");
          Serial.println(int_result);

          size_t arraySize = sizeof(int);

          uint8_t uint8Array[arraySize];

          intToUint8Array(int_result, uint8Array, arraySize);

          payload[1] = uint8Array[0];
          payload[2] = uint8Array[1];
          payload[3] = uint8Array[2];

          printHeaderAndData();
          return;          
        }

        if (strcmp(substring2, "off\0") == 0){
          
          header = 3;
          payload[1] = 0;
          payload[2] = 0;
          payload[3] = 0;

          printHeaderAndData();
          return;
        }

        char substring4[8] = {
          serialReceivedChars[8],
          serialReceivedChars[9],
          serialReceivedChars[10],
          serialReceivedChars[11],
          serialReceivedChars[12],
          serialReceivedChars[13],
          serialReceivedChars[14],
          '\0'
          };

        if (strcmp(substring4, "unit in\0") == 0){
          
          header = 4;
          payload[1] = 0;
          payload[2] = 0;
          payload[3] = 0;

          printHeaderAndData();
          return;
        }

        if (strcmp(substring4, "unit cm\0") == 0){
          
          header = 5;
          payload[1] = 0;
          payload[2] = 0;
          payload[3] = 0;

          printHeaderAndData();
          return;
        }

        if (strcmp(substring4, "unit ms\0") == 0){
          
          header = 6;
          payload[1] = 0;
          payload[2] = 0;
          payload[3] = 0;

          printHeaderAndData();
          return;
        }    

        char substring5[7] = {
          serialReceivedChars[8],
          serialReceivedChars[9],
          serialReceivedChars[10],
          serialReceivedChars[11],
          serialReceivedChars[12],
          serialReceivedChars[13],
          '\0'
          };

        if (strcmp(substring5, "delay ") == 0){
          
          header = 7;
          
          uint8_t i=14;
          char substring3[32];
          while (serialReceivedChars[i] != '\0'){
            substring3[i-14] = serialReceivedChars[i];
            i++;
          }

          int int_result = strtol(substring3, &output, 10);

          Serial.print("Stored number: ");
          Serial.println(int_result);

          size_t arraySize = sizeof(int);

          uint8_t uint8Array[arraySize];

          intToUint8Array(int_result, uint8Array, arraySize);

          payload[1] = uint8Array[0];
          payload[2] = uint8Array[1];
          payload[3] = uint8Array[2];

          printHeaderAndData();
          return;
        }      


        if (strcmp(substring5, "status\0") == 0){
          
          header = 8;
          payload[1] = 0;
          payload[2] = 0;
          payload[3] = 0;

          printHeaderAndData();
          return;
        }                     

      }
    }
    Serial.println("Use 'help' to view available commands");
    serialNewData = false;
  }
}

void sendingData() {

  if (serialNewData == true) {
    Serial.println("sending..."); 

    //DESCOMENTAR
    //////////////////////////////////////////////////////////////////
    // call second function to write option code on Serial1
    while (serial1WriteHeader() == false){}
    
    // call second function to write data on Serial1
    while (serial1WriteData() == false){}
    //////////////////////////////////////////////////////////////////
    
    serialNewData = false;
    serialReceivedChars[NUM_CHARS] = {};
  }
}

bool serial1WriteHeader(){

  Serial.print("Sending header--> ");

  Serial1.write(header);

  acknowledge = 0;
  last_ms=millis();
  new_ms=millis();
  // Waiting for the Option acknowledge
  while(new_ms-last_ms<1000 && acknowledge != 255) 
    { 
      if(Serial1.available()>0)
      {
        acknowledge=Serial1.read();
        Serial.print("<-- ack received: "); Serial.println(static_cast<int>(acknowledge));
        return true;
      }
      new_ms=millis();
    }
  return false;
}

bool serial1WriteData(){
  
  Serial.print("Sending payload--> ");

  Serial1.write(payload[0]);
  Serial1.write(payload[1]);
  Serial1.write(payload[2]);
  Serial1.write(payload[3]);

  acknowledge = 0;
  last_ms=millis();
  new_ms=millis();
  // Waiting for the Option acknowledge
  while(new_ms-last_ms<1000 && acknowledge != 255) 
    { 
      if(Serial1.available()>0)
      {
        uint8_t acknowledge=Serial1.read();
        Serial.print("<-- ack received: "); Serial.println(static_cast<int>(acknowledge)); 
        return true;
      }
      new_ms=millis();
    }
  return false;
}

void printHeaderAndData(){
  Serial.println(header);
  Serial.println(payload[0]);
  Serial.println(payload[1]);
  Serial.println(payload[2]);
  Serial.println(payload[3]);
}

void intToUint8Array(int value, uint8_t* array, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        array[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

#else

// IS SLAVE MODE

////////////////
/// Protocol ///
////////////////

  void checkSerial1(){

    if(Serial1.available()>0){
        
      header = Serial1.read();
      rec = true;

      // ToDo: Detect which command was queried
      /*
      /
      /
      / 
      /
      */
      Serial.println(header);

    }

    if (rec == true){
      Serial.println("Sending acknowledge--> ");
      Serial1.write(0xFF);

      last_ms=millis();
      new_ms=millis();
      while (new_ms-last_ms<1000){
        
        if (Serial1.available()>0) {
          data[counter] = Serial1.read();
          counter++;
        }
        new_ms=millis();
      }

      counter = 0;

      rec = false;

      Serial.println(data[0]);
      Serial.println(data[1]);
      Serial.println(data[2]);
      Serial.println(data[3]);

      Serial1.write(0xFF);
    }
  }

  // Function to send data to the master device using Serial1
  void sendDataToMaster(uint8_t* data, size_t dataSize) {
   /*
   /  ToDo: Send data to master
   */    
  }

////////////////////
/// OLED Display ///
////////////////////

  // Take the two 16 bit int from the sensor and display them on the OLED
  void displayDataToOLED(uint16_t range, uint16_t minRange) {
    display.clearDisplay();

    display.setTextSize(1); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    // Display range
    display.setCursor(10, 0);
    display.print(F("Distance: "));
    display.print(range);
    display.println(F(" cms"));
    // Display min range (autotune)
    display.setCursor(10, display.getCursorY());      // Offset curser towards center of the screen
    display.print(F("Min range: "));
    display.print(minRange);
    display.println(F(" cms"));
    // Display remaining time to measure
    // Or display request status
    if (_is_periodic_ranging) {
      display.setCursor(10, display.getCursorY());
      display.println("Measuring for: ");
      display.setCursor(10, display.getCursorY());
      display.print(sensorPeriod_ms);
      display.println("ms");
      display.setCursor(10, display.getCursorY());
      display.println("Remaining time: ");
      display.setCursor(10, display.getCursorY());
      display.print(sensorPeriod_ms - sensorFirstTime - millis());
      display.println("ms");
    }else {
      display.setCursor(10, display.getCursorY());
      display.println("No request! ");
      display.setCursor(10, display.getCursorY());
      display.println("Waiting... ");
    }
    // Show initial text
    display.display();
  }

  //////////////
  /// Sensor ///
  //////////////

  // Read sensor once, LED_BUILTIN is lit during ranging process
  void com_srfOneShot(){
    digitalWrite(LED_BUILTIN, HIGH);
    write_command(sensorAddress,REAL_RANGING_MODE_CMS);
      // Delay to make accurate reading -> see manual
      delay(SRF02_RANGING_DELAY);

      byte high_byte_range=read_register(SRF02_I2C_ADDRESS,RANGE_HIGH_BYTE);
      byte low_byte_range=read_register(SRF02_I2C_ADDRESS,RANGE_LOW_BYTE);
      byte high_min=read_register(SRF02_I2C_ADDRESS,AUTOTUNE_MINIMUM_HIGH_BYTE);
      byte low_min=read_register(SRF02_I2C_ADDRESS,AUTOTUNE_MINIMUM_LOW_BYTE);

      srf_range = int16_t((high_byte_range<<8) | low_byte_range);
      srf_min = int16_t((high_min<<8) | low_min);
      digitalWrite(LED_BUILTIN, LOW);
  }

  void com_srfOnPeriod_ms(uint16_t period_ms) {
    _is_periodic_ranging = true;
    sensorPeriod_ms = period_ms;
    sensorFirstTime = millis();         // Set starting time for periodic sensor reading
    sensorLastTime = sensorFirstTime;   // Reset variable
  }

  // USING tc_lib
  //void sensor_callback(void* a_ctx) {
  //  ctx* the_ctx=reinterpret_cast<ctx*>(a_ctx);
  //
  //  //Call sensor reading
  //  com_srfOneShot();
  //
  //  // Increase counter for timer
  //  the_ctx->counter++;
  //  // If period is reached, stop timer callback
  //  if (millis() - the_ctx->start > sensorPeriod_ms) {
  //    // DEBUGGING
  //    SerialUSB.println("Stopping timer now");
  //    action_tc4.stop();
  //  }
  //}

  // Take all available inforamtion suh as sensor address, configured delay and period as well as selected unit format and send information to master
  void com_retrieveStatus(){
    int arr_size = 7;
    uint8_t rspData[arr_size];
    rspData[0] = sensorAddress & 0xFF;          // Lower 8 bits of sensorAddress
    rspData[1] = (sensorAddress >> 8) & 0xFF;   // Upper 8 bits of sensorAddress

    rspData[2] = sensorDelay_ms & 0xFF;         // Lower 8 bits of sensorDelay_ms
    rspData[3] = (sensorDelay_ms >> 8) & 0xFF;  // Upper 8 bits of sensorDelay_ms

    rspData[4] = sensorPeriod_ms & 0xFF;        // Lower 8 bits of sensorPeriod_ms
    rspData[5] = (sensorPeriod_ms >> 8) & 0xFF; // Upper 8 bits of sensorPeriod_ms

    rspData[6] = sensorUnit;                    // Already uint8_t

    sendDataToMaster(rspData, arr_size);
  }

  // Set delay between distinct sensor readings
  void com_setDelay(uint16_t address, uint16_t delay){
    sensorAddress = address;
    sensorDelay_ms = delay;
  }

  // Set unit for sensor reading
  // Will halt if unit can not be determined;
  void com_setUnit(uint16_t address, uint8_t unit){
    sensorAddress = address;
    switch (unit) {
      case CMS:
        sensorUnit = REAL_RANGING_MODE_CMS;
      case INC:
        sensorUnit = REAL_RANGING_MODE_INCHES;
      case USEC:
        sensorUnit = REAL_RANGING_MODE_USECS;
      default:
        SerialUSB.println("Error: could not determine measuring unit for sensor. Halting...");
        while(1){
          digitalWrite(LED_BUILTIN, LOW);
          delay(100);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(100);
        }
    }
  }

#endif


