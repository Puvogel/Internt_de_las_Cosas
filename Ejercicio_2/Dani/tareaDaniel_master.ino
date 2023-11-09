/* ----------------------------------------------------------------------
 *  Ejemplo sending_example.ino 
 *    Este ejemplo muestra como utilizar el puerto serie uart (Serial1) 
 *    para comunicarse con otro dispositivo.
 *    
 *  Asignatura (GII-IC)
 * ---------------------------------------------------------------------- 
 */

constexpr const uint32_t serial_monitor_bauds=115200;
constexpr const uint32_t serial1_bauds=9600;

constexpr const uint32_t pseudo_period_ms=1000;

const byte numChars = 32;
char serialReceivedChars[numChars];   // an array to store the received command

uint8_t opbyte=0;

boolean serialNewData = false;

void setup()
{
  // Inicialización del puerto para el serial monitor 
  Serial.begin(serial_monitor_bauds);
  while (!Serial);

  // Inicialización del puerto de comunicaciones con el otro dispositivo MKR 
  Serial1.begin(serial1_bauds);

  Serial.println("Use 'help' to view available commands");
  
}

void loop()
{
  
  inputData();          // Checks if there is any new data to be read from the serial monitor
  showData();           // Shows the new data if there is any
  checkData();          // checks for any syntax error or 'help' command
  sendDataToSlave();    // Send the data to the Slave
  
 
  /*
  
  Serial.println("******************* sending example *******************"); 

  Serial.print("--> sending: "); Serial.println(static_cast<int>(counter)); 
  Serial1.write(counter++);

  uint32_t last_ms=millis();
  while(millis()-last_ms<pseudo_period_ms) 
  { 
    if(Serial1.available()>0) 
    {
      uint8_t data=Serial1.read();
      Serial.print("<-- received: "); Serial.println(static_cast<int>(data)); 
      break;
    }
  }

  if(millis()-last_ms<pseudo_period_ms) delay(pseudo_period_ms-(millis()-last_ms));
  else Serial.println("<-- received: TIMEOUT!!"); 

  Serial.println("*******************************************************"); 

  digitalWrite(LED_BUILTIN,led_state); led_state=(led_state+1)&0x01;
  */
}


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
