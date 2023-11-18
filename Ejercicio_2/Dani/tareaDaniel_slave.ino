constexpr const uint32_t serial_monitor_bauds=115200;
constexpr const uint32_t serial1_bauds=9600;

constexpr const uint32_t pseudo_period_ms=1000;

uint8_t led_state=LOW;

const byte numChars = 32;
char serial1ReceivedChars[numChars];   // an array to store the received command

static byte ndx = 0;
char endMarker = '\n';
char rc;

uint8_t counter = 0;
uint8_t header = 0;

bool rec = false;

uint8_t data[3] = {};

uint32_t last_ms;
uint32_t new_ms;

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

  checkSerial1();

}

void checkSerial1(){

  if(Serial1.available()>0){
      
    header = Serial1.read();
    rec = true;
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


