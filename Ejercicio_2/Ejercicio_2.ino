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



// Connect OLED cables to the following port
// red    -> 3.3V/5V
// black  -> GND
// yelow  -> scl
// blue   -> sda


#define IS_MASTER false    // Set Master mode for controlling device, for sensor device set to false


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
  boolean isActiveRanging = false;  // Semaphor variable to regulate ranging calls

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

#endif

void setup() {

  // Setup indicator LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup serial connection
  // SerialUSB.begin(115200);

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

    //checkSerial1();     // Checks if there is any new data to be read from the serial1
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

    // Measure after SRF02_MEASURE_DELAY ms
    if (millis() - lastTimeSendor_ms > SRF02_MEASURE_DELAY && !isActiveRanging) {
      write_command(SRF02_I2C_ADDRESS,REAL_RANGING_MODE_CMS);
      // Delay to make accurate reading -> see manual
      delay(SRF02_RANGING_DELAY);

      byte high_byte_range=read_register(SRF02_I2C_ADDRESS,RANGE_HIGH_BYTE);
      byte low_byte_range=read_register(SRF02_I2C_ADDRESS,RANGE_LOW_BYTE);
      byte high_min=read_register(SRF02_I2C_ADDRESS,AUTOTUNE_MINIMUM_HIGH_BYTE);
      byte low_min=read_register(SRF02_I2C_ADDRESS,AUTOTUNE_MINIMUM_LOW_BYTE);

      srf_range = int16_t((high_byte_range<<8) | low_byte_range);
      srf_min = int16_t((high_min<<8) | low_min);

      // Reset timer variable
      lastTimeSendor_ms = millis();
    }

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

void checkSerial1() {
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
    // Show initial text
    display.display();
    //delay(100);   // Do not pause in subroutines

    // Scroll in various directions, pausing in-between:
    /*display.startscrollright(0x00, 0x0F);
    delay(2000);
    display.stopscroll();
    delay(1000);
    display.startscrollleft(0x00, 0x0F);
    delay(2000);
    display.stopscroll();
    delay(1000);
    display.startscrolldiagright(0x00, 0x07);
    delay(2000);
    display.startscrolldiagleft(0x00, 0x07);
    delay(2000);
    display.stopscroll();
    delay(1000);*/
  }

#endif


