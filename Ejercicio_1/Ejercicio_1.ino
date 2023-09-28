/*
* This code uses snippets and functions derived from "Internet de las Cosas" at ULPGC
* Code that was not written by myself is marked by a comment "adapted" in the function desciption
* 
* This code is part of the first assignment for the course on "Internet de las Cosas" at ULPGC
*
* Author: Karl Deilmann
*/

// Importing Real-tiem clock library
#include <RTCZero.h>
#include <time.h> 
#include <ArduinoLowPower.h>
#include <Arduino_MKRMEM.h> // ATTENTION: In "~/src/Arduino_W25Q16DV.cpp" Line 193-199 is commented out to work with internal flash memory
Arduino_W25Q16DV flash(SPI1, FLASH_CS);

// Define macros
//#define elapsedMilliseconds(since_ms) (uint32_t)(millis() - since_ms)   // Return elapes time in ms // Adapted
#define SIZE_OF_DATE_TIME 64


// Declare variables
RTCZero rtc;                                // Define RTC
volatile uint32_t _period_sec = 0;          // Define update period for RTC alarm
char dateTime[SIZE_OF_DATE_TIME];           // Global DateTime data

// Define const vaiables
const char filename[] = "sensorData.txt";   // Define filename for a file to store sensor data (NOTE: in Task 3 we write timestamps instead of sensordata)
const int externalPin = 5;                  // Define ext.  pin for interactions by the outside world
const int limit = 5;                       // Define limit loops

// Define flags
const bool verboseSerial = true;            // Define verbosit of SerialMonitor output
volatile uint8_t _rtcFlag = 0;              // Define alarm flag for RTC alarm
volatile uint8_t _extInterrupt = 0;         // Define external interrupt flag



void setup() {

  // Setup indicator LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup serial connection
  SerialUSB.begin(115200);

  // Validate connection
  //validateConnection();
  // SerialUSB.println("setup alive"); // DEBUGGING

  /* Task 1: Setup of RTC */
  // Setup RTCZero rtc
  setupRTC();

  /* Task 3: Setup for FLASH memory */
  // Deactivate LoRa module, since it does not work with the altered <Arduino_MKRMEM.cpp> code
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);

  // Setup FLASH memory
  setupFLASH();

  // Setup files that we will late write to
  setupFiles();

  /*if(verboseSerial){
    SerialUSB.println("Task 2: Start RTC alarm 10 Sec, 3 Sec offset");
  }*/

  // Setup ext. pin to pull-up
  pinMode(externalPin, INPUT_PULLUP);
  // Subscribe ISR to external interrupt event on edge
  LowPower.attachInterruptWakeup(externalPin, externalCallback, FALLING);

  // Subscribe alarmCallback() as ISR to wakeup Arduino
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmCallback, CHANGE);
  // Set RTC alarm
  setPeriodicAlarm(5, 3);
  // Reset flag to 0
  _rtcFlag = 0;

  /* Only for Task 2
  // Determine callback function to call on alarm interrupt
  rtc.attachInterrupt(alarmCallback);
  */

  // Promt OK to seial monitor
  if(verboseSerial){
    SerialUSB.println("Setup OK\nSleeping now...");
  }

  // Send Arduino to sleep (indefinetely)
  LowPower.sleep();
}

void loop() {

  static int repetition = 0;    // For Debugging

  SerialUSB.println("Loop Alive");

  // Task 5: Only show bahaviour form Task 2 on ext. interrupt
  if (_extInterrupt) {
    digitalWrite(LED_BUILTIN, HIGH);    // FOR DEBUGGING
    delay(100);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, LOW);     // FOR DEBUGGING
    delay(100);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, HIGH);    // FOR DEBUGGING
    delay(100);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, LOW);     // FOR DEBUGGING
    delay(100);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, HIGH);    // FOR DEBUGGING
    delay(100);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, LOW);     // FOR DEBUGGING
    // Print ext. interrupt notice and dateTime
    SerialUSB.println("External interrupt detected...");
    printDateTime();
    strcat(dateTime, " external\n");
    writeDataToFLASH();


    // Decrement flag
    _extInterrupt--;
    if (verboseSerial) {
      if(_extInterrupt != 0) SerialUSB.println("Unexpected behaviour: _extInterrupt not 0 after decrement. Runnaway detected?");
    }
  }
  
  /* Task 2: Use RTC to read sensor every 10 seconds. Print out date and time on each read */
  if(_rtcFlag){
    digitalWrite(LED_BUILTIN, HIGH);    // FOR DEBUGGING
    delay(200);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, LOW);     // FOR DEBUGGING
    delay(200);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, HIGH);    // FOR DEBUGGING
    delay(200);                         // FOR DEBUGGING
    digitalWrite(LED_BUILTIN, LOW);     // FOR DEBUGGING
    // Print date and time
    printDateTime();
    strcat(dateTime, " internal\n");
    writeDataToFLASH();
    /////// Read sensor routine here ////////////////
    // Decrement _rtcFlag
    _rtcFlag--;
    if (verboseSerial) {
      if(_rtcFlag != 0) SerialUSB.println("Unexpected behaviour: _rtcFlag not 0 after decrement. Runnaway detected?");
    }
    repetition++;

    // FOR DEBUGGING
    if (repetition > limit) {
      getFSInfo();
      readFlash();
      exit(0);
    }
  }

  if (repetition < limit) {
    SerialUSB.println("Sleep from loop...");
    // Send Arduino to sleep (indefinetely)
    LowPower.sleep();
  }
}

// Check serial connection
// Will flash indicator LED while connection is not estabilshed
// On success, promt to serial monitor
bool validateConnection() {
  while(!SerialUSB){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  if(verboseSerial){
    SerialUSB.println("Connection OK...\nContinuing with Setup...");
  }
  return true;
}

// Start RTC and set time to compile-time in human readable form
bool setupRTC() {
  // start RTC
  rtc.begin();

  // Set RTC to compile date/time, return error on serial monitor on failure
  if (!setDateTime(__DATE__, __TIME__))
  {
    if(verboseSerial){
      SerialUSB.println("setDateTime() failed!\nExiting ...\nManual reset requiered");
    }
    // Enter infinite loop
    while (1) {
      // Do nothing
    }
  }
  return true;
}

// Initial Flash memory setup routine
// Erase flash module and prepare memory for usage
// partly adapted
bool setupFLASH() {
  flash.begin();

  // Clear memory
  if(verboseSerial){
    SerialUSB.println("Clearing memory chip...");
  }
  flash.eraseChip();
  if(verboseSerial){
    SerialUSB.println("Memory erased...");
  }

  // Mount filesystem
  if(verboseSerial){
    SerialUSB.println("Mounting filesystem...");
  }
  // Mount and check if mounting OK
  int res = safeMount();

  // Unmount and format memory
  if(verboseSerial){
    SerialUSB.println("Unmounting\nPrepare formatting");
  }
  filesystem.unmount();
  res = filesystem.format();
  if(res != SPIFFS_OK){
    if(verboseSerial) {
      SerialUSB.println("format() failed with error code ");
      SerialUSB.println(res);
    }
    return false;
  }

  // Mount fs again
  if(verboseSerial){
    SerialUSB.println("Mounting ...");
  }
  res = safeMount();
  
  // Checking fs for errors
  if(verboseSerial){
    SerialUSB.println("Checking ...");
  }
  res = safeCheck();
  






  // Read and print out info on used memory
  getFSInfo();

  // Unmounting fs
  if(verboseSerial){
    SerialUSB.println("Unmounting ... formatting finished!");
  }
  filesystem.unmount();

  // All OK
  return true;
}

void writeDataToFLASH(){
  /* Task 3: Save Serial output to external flash memory*/

    File file = filesystem.open(filename, WRITE_ONLY | APPEND);
    // Check if file is valid
    if(!file){
      if(verboseSerial){
        SerialUSB.print("Opening file ");
        SerialUSB.print(filename);
        SerialUSB.print(" failed for appending. Aborting ...");
      }
      on_exit_with_error_do();
    }

    int const bytes_to_write = strlen(dateTime);
    int const bytes_written = file.write((void *)dateTime, bytes_to_write);

    // Check whether the intended amount of bytes have been written
    if (bytes_to_write != bytes_written) {
      if (verboseSerial) {
        SerialUSB.print("write() failed with error code ");
        SerialUSB.println(filesystem.err());
        SerialUSB.println("Aborting ...");
      }
      on_exit_with_error_do();
    }

    // Close file
    file.close();
}

// Mount FS and check success
int safeMount() {
  int res = filesystem.mount();
  if(res != SPIFFS_OK && res != SPIFFS_ERR_NOT_A_FS){
    if(verboseSerial) {
      SerialUSB.println("mount() failed with error code ");
      SerialUSB.println(res); 
    }
    on_exit_with_error_do();
  }
  return res;
}

int safeCheck() {
  int res = filesystem.check();
  if(res != SPIFFS_OK){
    if(verboseSerial) {
      SerialUSB.println("check() failed with error code ");
      SerialUSB.println(res);
    }
  }
  return res;
}

// Retreive fs info and rint it to serial
// USE ONLY on mounted FS
bool getFSInfo() {
  if(verboseSerial){
    SerialUSB.println("Retrieving filesystem info ...");
  }
  unsigned int bytes_total = 0,
  bytes_used = 0;
  int res = filesystem.info(bytes_total, bytes_used);
  if(res != SPIFFS_OK) {
    if(verboseSerial){
      SerialUSB.println("check() failed with error code ");
      SerialUSB.println(res); 
    }
    return false;
  } else {
    if(verboseSerial){
      char msg[64] = {0};
      snprintf(msg, sizeof(msg), "SPIFFS Info:\nBytes Total: %d\nBytes Used: %d", bytes_total, bytes_used);
      SerialUSB.println(msg);
    }
  }
}


// Create files on FS for later use
void setupFiles() {
  safeMount();
  // Creating (or truncate if exists) new file with <filename> 
  File file = filesystem.open(filename, CREATE | TRUNCATE);
  if(!file){
    if (verboseSerial) {
      SerialUSB.print("Creation of file ");
      SerialUSB.print(filename);
      SerialUSB.print(" failed. Aborting ...");
    }
    on_exit_with_error_do();
  }
  file.close();
}


// Routine to call in case of unresolvable error
// Performs cleanup
void on_exit_with_error_do() {
 filesystem.unmount();
 exit(EXIT_FAILURE);
}

// Read the content of flash memory
void readFlash(){
  File file = filesystem.open(filename, READ_ONLY);
      if (!file) {
        if (verboseSerial) {
          SerialUSB.print("Opening file ");
          SerialUSB.print(filename);
          SerialUSB.print(" failed for reading. Aborting ...");
        }
        on_exit_with_error_do();
      }
      if (verboseSerial) {
        SerialUSB.print("Reading file contents:\n\t ");
      }
      // LRead till EOF and close the file
      while(!file.eof()) {
        char c;
        int const bytes_read = file.read(&c, sizeof(c));
        if (bytes_read) {
          SerialUSB.print(c);
          if (c == '\n') SerialUSB.print("\t ");
        }
      }
      // Close file
      file.close();
      if (verboseSerial) {
        SerialUSB.println("\nFile closed");
      }
}

// Set the time in human readable format
// Adapted
bool setDateTime(const char * date_str, const char * time_str) {
 char month_str[4];
 char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug",
 "Sep", "Oct", "Nov", "Dec"};
 uint16_t i, mday, month, hour, min, sec, year;
 if (sscanf(date_str, "%3s %hu %hu", month_str, &mday, &year) != 3) return false;
 if (sscanf(time_str, "%hu:%hu:%hu", &hour, &min, &sec) != 3) return false;
    for (i = 0; i < 12; i++) {
          if (!strncmp(month_str, months[i], 3)) {
          month = i + 1;
          break;
          }
    }
 if (i == 12) return false;
 rtc.setTime((uint8_t)hour, (uint8_t)min, (uint8_t)sec);
 rtc.setDate((uint8_t)mday, (uint8_t)month, (uint8_t)(year - 2000));
 return true;
}

// Print out current RTC time in human readable form
void printDateTime() {
 const char *weekDay[7] = { "Sun", "Mon", "Tue", "Wed", "Thr", "Fri", "Sat" };

 // Get current time
 time_t epoch = rtc.getEpoch();
 // Convet time_t to date formate
 struct tm stm;
 gmtime_r(&epoch, &stm);

 // Generate dateTime string
 snprintf(dateTime, sizeof(dateTime),"%s %4u/%02u/%02u %02u:%02u:%02u", weekDay[stm.tm_wday], stm.tm_year + 1900, stm.tm_mon + 1, stm.tm_mday, stm.tm_hour, stm.tm_min, stm.tm_sec);
 // Print out dateTime
 SerialUSB.println(dateTime);
}


// Initialize RTCZero alarm that is triggered every <periodic_sec> after an initial offset value in seconds
// Adapted
void setPeriodicAlarm(uint32_t period_sec, uint32_t offsetFromNow_sec) {
 _period_sec = period_sec;
 rtc.setAlarmEpoch(rtc.getEpoch() + offsetFromNow_sec);
 rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

// Service routine triggered by the RTC interrupt
// Sets next alarm after _period_sec time
// Adapted
void alarmCallback() {
 // Increment flag
 _rtcFlag++;
 // Schedule next RTC alarm 
 rtc.setAlarmEpoch(rtc.getEpoch() + _period_sec);
}

// ISR to be called on ext. interrupt event
void externalCallback(){
  // Increment flag
  _extInterrupt++;
}



