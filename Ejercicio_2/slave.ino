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

void slaveSetup(){
  // DEBUGGING
  /*_is_periodic_ranging = true;
  SerialUSB.println("DEBUGGING: Strating period sensor reading once...");*/

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
  /*Serial.println("Begining Sensor setup ...");
  Serial.println("initializing Wire interface ...");
  Wire.begin();
  delay(SRF02_I2C_INIT_DELAY);  

  byte software_revision=read_register(SRF02_I2C_ADDRESS,SOFTWARE_REVISION);
  Serial.print("SFR02 ultrasonic range finder in address 0x");
  Serial.print(SRF02_I2C_ADDRESS,HEX); Serial.print("(0x");
  Serial.print(software_revision,HEX); Serial.println(")");

  Serial.println("Finished Sensor setup ...");*/
}


void slaveLoop(){
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
    //Serial.println("Refresh OLED ...");
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
      Serial.println("Reached  P1.2 ...");
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
}



/// Additional functions ///

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

    display.setTextSize(1); // Draw 1X-scale text
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