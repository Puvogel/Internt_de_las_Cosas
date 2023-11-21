uint8_t payload[4] = {};
bool temp = false;

uint8_t acknowledge;

uint8_t result;
char *output;
char serialReceivedChars[NUM_CHARS];


void masterSetup(){
  Serial.println("Use 'help' to view available commands");
}

void masterLoop(){
  //inputData();          // Checks if there is any new data to be read from the serial monitor
  showData();           // Shows the new data if there is any
  checkData();          // checks for any syntax error or 'help' command
  //sendDataToSlave();    // Send the data to the Slave
}

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