/* ----------------------------------------------------------------------
 *  Ejemplo echo.ino 
 *    Este ejemplo muestra como utilizar el puerto serie uart (Serial1) 
 *    para comunicarse con otro dispositivo.
 *    
 *  Asignatura (GII-IC)
 * ---------------------------------------------------------------------- 
 */

constexpr const uint32_t serial_monitor_bauds=115200;
constexpr const uint32_t serial1_bauds=9600;

constexpr const uint32_t pseudo_period_ms=1000;

uint8_t led_state=LOW;

const byte numChars = 32;
char serial1ReceivedChars[numChars];   // an array to store the received command

static byte ndx = 0;
char endMarker = '\n';
char rc;

boolean serial1NewData = false;

void setup()
{  
  // Inicialización del puerto para el serial monitor 
  Serial.begin(serial_monitor_bauds);
  while (!Serial);

  // Inicialización del puerto de comunicaciones con el otro dispositivo MKR 
  Serial1.begin(serial1_bauds);

  Serial.println("waiting..."); 
}

void loop()
{
  
  checkSerial1();     // Checks if there is any new data to be read from the serial1
  showData();         // Shows the new data if there is any
  
  //acknowledge();
  /*
  uint32_t last_ms=millis();
  while(millis()-last_ms<pseudo_period_ms) 
  { 
    if(Serial1.available()>0) 
    {
      uint8_t data=Serial1.read();
      Serial.print("<-- received: "); Serial.println(static_cast<int>(data)); 
      Serial.print("--> sending back: "); Serial.println(static_cast<int>(data)); 
      Serial1.write(data);
      break;
    }
  }

  if(millis()-last_ms<pseudo_period_ms) delay(pseudo_period_ms-(millis()-last_ms));
  else Serial.println("<-- received: TIMEOUT!!"); 

  Serial.println("*******************************************************"); 

  digitalWrite(LED_BUILTIN,led_state); led_state=(led_state+1)&0x01;
  */
}

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
