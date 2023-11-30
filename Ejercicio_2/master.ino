uint8_t payload[4] = {};
bool temp = false;

bool slaveDataFromSrf02 = false;

uint8_t acknowledge;

uint8_t dataSize = 0;

const byte numChars = NUM_CHARS;

uint8_t result;
char *output;
char serialReceivedChars[NUM_CHARS];

uint16_t int_period_result;

bool periodic_read_on = false;

int int_delay_result = 70;


void masterSetup(){

  Serial.begin(serial_monitor_bauds);
  while (!Serial);
  Serial.println("Use 'help' to view available commands");
}

void masterLoop(){
  checkSerial();         // Checks if there is any new data to be read from the serial monitor
  showData();            // Shows the new data if there is any
  checkData();           // checks for any syntax error or 'help' command
  sendingData();         // Send the data to the Slave
  delay(1500);
  receivingData();       // 
}

void checkSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0) {
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
        header = CODE_US;
        payload[0] = 0;
        payload[2] = 0;
        payload[1] = 0;

        slaveDataFromSrf02 = true;
        // receiving 2 bytes from the addresses of the sensors
        dataSize = 2; 

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

        if (periodic_read_on == false){
          if (strcmp(substring1, "one-s\0") == 0){
            
            
            header = CODE_ONE_SHOT;
            payload[2] = 0;
            payload[1] = 0;

            // receiving 4 bytes from slave from the one shot
            dataSize = 4;

            slaveDataFromSrf02 = true;

            

            printHeaderAndData();
            return;
          }
        }



        char substring2[4] = {
          serialReceivedChars[8],
          serialReceivedChars[9],
          serialReceivedChars[10],
          '\0'
          };

        if (strcmp(substring2, "on \0") == 0){
          
          header = CODE_ON_PERIOD_MS;
          periodic_read_on = true;


          uint8_t i=11;
          char substring3[32];
          while (serialReceivedChars[i] != '\0'){
            substring3[i-11] = serialReceivedChars[i];        
            i++;
          }
          substring3[i-11] = '\0';

          int_period_result = strtol(substring3, &output, 10);

          Serial.print("Stored number: ");
          if (int_period_result < int_delay_result){
            int_period_result = int_delay_result;
            Serial.println(int_period_result);
            Serial.println("Period is smaller than delay so setting to same value as delay");
          }
          else {
            Serial.println(int_period_result);
          }


          payload[2] = static_cast<uint8_t>(int_period_result & 0xFF);
          payload[1] = static_cast<uint8_t>((int_period_result >> 8) & 0xFF);
          
          // receiving 4 bytes from slave from the periodic shot
          dataSize = 4;          

          slaveDataFromSrf02 = true;

          printHeaderAndData();
          return;          
        }

        if (strcmp(substring2, "off\0") == 0){
          
          header = CODE_OFF;
          periodic_read_on = false;

          payload[2] = 0;
          payload[1] = 0;
          

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
          
          header = CODE_UNIT;
          payload[1] = CODE_UNIT_INC;
          payload[2] = 0;

          printHeaderAndData();
          return;
        }

        if (strcmp(substring4, "unit cm\0") == 0){
          
          header = CODE_UNIT;
          payload[1] = CODE_UNIT_CMS;
          payload[2] = 0;
          

          printHeaderAndData();
          return;
        }

        if (strcmp(substring4, "unit ms\0") == 0){
          
          header = CODE_UNIT;
          payload[1] = CODE_UNIT_MS;
          payload[2] = 0;
          

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
          
          header = CODE_DELAY;
          
          uint8_t i=14;
          char substring3[32];
          while (serialReceivedChars[i] != '\0'){
            substring3[i-14] = serialReceivedChars[i];
            i++;
          }

          int_delay_result = strtol(substring3, &output, 10);

          Serial.print("Stored number: ");
          if (int_delay_result < 70){ 
            Serial.println("value to small");
            Serial.println("Defaulting to 70");
            int_delay_result = 70;
          }
          else {
            Serial.println(int_delay_result);
          }
          

          size_t arraySize = sizeof(int);

          uint8_t uint8Array[arraySize];

          intToUint8Array(int_delay_result, uint8Array, arraySize);

          payload[2] = uint8Array[0];
          payload[1] = uint8Array[1];

          printHeaderAndData();
          return;
        }      

        if (periodic_read_on == false){
          if (strcmp(substring5, "status\0") == 0){
            
            header = CODE_STATUS;
            payload[2] = 0;
            payload[1] = 0;

            // receiving 7 bytes from slave from the status
            dataSize = 7;

            slaveDataFromSrf02 = true;

            printHeaderAndData();
            return;
          }                     
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
    
    serial1WriteHeader();
    
    
    // call second function to write data on Serial1
    serial1WriteData();
    //////////////////////////////////////////////////////////////////
    

    serialReceivedChars[numChars] = {};
    delay(1500);
  }
}


void serial1WriteHeader(){

  Serial.print("Sending header: "); Serial.println(header);

  Serial1.write(header);
  Serial1.flush();

}


void serial1WriteData(){
  
  Serial.print("Sending payload: ");
  Serial.print(payload[0]);
  Serial.print(", ");
  Serial.print(payload[1]);
  Serial.print(", ");
  Serial.println(payload[2]);

  Serial1.write(payload[0]);
  Serial1.write(payload[1]);
  Serial1.write(payload[2]);

  Serial1.flush();


  
/*
  acknowledge = 0;

  // Waiting for the Option acknowledge
  if(Serial1.available()>0)
  {
    uint8_t acknowledge=Serial1.read();
    if (acknowledge == CODE_ACK)
    {
      Serial.print("right acknowledge: "); Serial.println(static_cast<int>(acknowledge));
    }
    else {
      Serial.print("wrong acknowledge: "); Serial.println(static_cast<int>(acknowledge));
    }
  }*/
  
}

void printHeaderAndData(){
  Serial.println(header);
  Serial.println(payload[0]);
  Serial.println(payload[1]);
  Serial.println(payload[2]);
}

void intToUint8Array(int value, uint8_t* array, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        array[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFF);
    }
}

void receivingData(){
  
  if (periodic_read_on == true){
    
    byte dataFromSlave[dataSize] = {};

    if(Serial1.available()>0){
      Serial1.readBytes(dataFromSlave, dataSize);
    

      uint16_t distance = (dataFromSlave[1] << 8) | dataFromSlave[0];
      uint16_t range = (dataFromSlave[3] << 8) | dataFromSlave[2];

      Serial.println("distancia: ");
      Serial.println(distance);
      Serial.println("rangeMin: ");
      Serial.println(range);
    }
    
    
  } else {
    if (serialNewData == true){
      if (header == CODE_ONE_SHOT){
        byte dataFromSlave[dataSize] = {};
          
        if(Serial1.available()>0){
          Serial1.readBytes(dataFromSlave, dataSize);
        }

        uint16_t range = (dataFromSlave[1] << 8) | dataFromSlave[0];
        uint16_t distance = (dataFromSlave[3] << 8) | dataFromSlave[2];

        Serial.println("distancia: ");
        Serial.println(distance);
        Serial.println("rangeMin: ");
        Serial.println(range);

      }
    
      if (header == CODE_US){
        byte dataFromSlave[dataSize] = {};
        byte dataFromSlave2[dataSize] = {};

        int i = 0;
        while(Serial1.available()>0){
          dataFromSlave2[i] = Serial1.read();
          i++;
          i= i%2;
        }
        
        uint8_t exchange=0;
        for(int j=0;j<dataSize;j++){
          dataFromSlave[j] = dataFromSlave2[(j + i)%2];
        }


        uint16_t range = (dataFromSlave[1] << 8) | dataFromSlave[0];

        Serial.println("addresses: ");
        Serial.println(dataFromSlave[0], HEX);
        Serial.println(dataFromSlave[1], HEX);
      }

      

      if (header == CODE_STATUS){
        byte dataFromSlave[dataSize] = {};
        byte dataFromSlave2[dataSize] = {};

        int i = 0;
        while(Serial1.available()>0){
          dataFromSlave2[i] = Serial1.read();
          i++;
          i= i%7;
        }
        
        uint8_t exchange=0;
        for(int j=0;j<dataSize;j++){
          dataFromSlave[j] = dataFromSlave2[(j + i)%7];
        }
        
        

        uint16_t sensorAdress = (dataFromSlave[1] << 8) | dataFromSlave[0];
        uint16_t sensorDelay = (dataFromSlave[3] << 8) | dataFromSlave[2];
        uint16_t sensorPeriod = (dataFromSlave[5] << 8) | dataFromSlave[4];

        Serial.println("Status: ");
        
        Serial.print("Sensor address: ");
        Serial.println(sensorAdress, HEX);
        Serial.print("Sensor delay: ");
        Serial.println(sensorDelay);
        Serial.print("Sensor period: ");
        Serial.println(sensorPeriod);
        Serial.print("Sensor unit: ");

        if (dataFromSlave[6] == 80) {Serial.println("Unit inches");}
        if (dataFromSlave[6] == 81) {Serial.println("Unit cms");}
        if (dataFromSlave[6] == 82) {Serial.println("Unit ms");}
        
      }

      /*
      if (header == CODE_ON_PERIOD_MS){
        last_ms=millis();
        new_ms=millis();
        byte dataFromSlave[dataSize] = {};
        while (new_ms-last_ms < int_period_result){
          if(Serial1.available()>0)
          {
            Serial1.readBytes(dataFromSlave, dataSize);
            Serial.println(dataFromSlave);
          }
          new_ms=millis();
        }
      }
      */
    }
  }
  serialNewData = false;
}