/* ---------------------------------------------------------------------
 *  Ejemplo MKR1310_LoRa_SendReceive_Binary
 *  Práctica 3
 *  Asignatura (GII-IoT)
 *  
 *  Basado en el ejemplo MKR1310_LoRa_SendReceive_WithCallbacks,
 *  muestra cómo es posible comunicar los parámetros de 
 *  configuración del transceiver entre nodos LoRa en
 *  formato binario *  
 *  
 *  Este ejemplo requiere de una versión modificada
 *  de la librería Arduino LoRa (descargable desde 
 *  CV de la asignatura.
 *  
 *  También usa la librería Arduino_BQ24195 
 *  https://github.com/arduino-libraries/Arduino_BQ24195
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS          25000

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0xC4;     // Dirección de este dispositivo
uint8_t destination = 0xBB;            // Dirección de destino, 0xFF es la dirección de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 4, 12, 8, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};

LoRaConfig_t oldNodeConf = { 4, 12, 8, 2};
LoRaConfig_t defaultNodeConf = { 4, 12, 8, 2};


int remoteRSSI = 0;
float remoteSNR = 0;

bool waitingToReceive = true;
bool requestDefaultMode = false;

bool defaultMode = true;


// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);  
  while (!Serial); 

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");
  
  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  // Configuramos algunos parámetros de la radio
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index])); 
                                  // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                  // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                  // Multiplicar por dos el ancho de banda
                                  // supone dividir a la mitad el tiempo de Tx
                                  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);     
                                  // [6, 12] Aumentar el spreading factor incrementa 
                                  // de forma significativa el tiempo de Tx
                                  // SPF = 6 es un valor especial
                                  // Ver tabla 12 del manual del SEMTECH SX1276
  
  LoRa.setCodingRate4(thisNodeConf.codingRate);         
                                  // [5, 8] 5 da un tiempo de Tx menor
                                  
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
                                  // Rango [2, 20] en dBm
                                  // Importante seleccionar un valor bajo para pruebas
                                  // a corta distancia y evitar saturar al receptor
  LoRa.setSyncWord(0xB2);         // Palabra de sincronización privada por defecto para SX127X 
                                  // Usaremos la palabra de sincronización para crear diferentes
                                  // redes privadas por equipos
  LoRa.setPreambleLength(8);      // Número de símbolos a usar como preámbulo

  
  // Indicamos el callback para cuando se reciba un paquete
  LoRa.onReceive(onReceive);
  
  // Activamos el callback que nos indicará cuando ha finalizado la 
  // transmisión de un mensaje
  LoRa.onTxDone(TxFinished);

  // Nótese que la recepción está activada a partir de este punto
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");
  
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() 
{
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;
  static uint32_t lastRestoreTime_ms = 0;
  

  if (!transmitting && !waitingToReceive) {
    
    // Slave has received the new conf from master 
    // so it will send message back to the master
    // with the new conf as an echo

    uint8_t payload[50];
    uint8_t payloadLength = 0;

    payload[payloadLength]    = (thisNodeConf.bandwidth_index << 4);
    payload[payloadLength++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
    payload[payloadLength]    = ((thisNodeConf.codingRate - 5) << 6);
    payload[payloadLength++] |= ((thisNodeConf.txPower - 2) << 1);

    // Incluimos el RSSI y el SNR del último paquete recibido
    // RSSI puede estar en un rango de [0, -127] dBm
    payload[payloadLength++] = uint8_t(-LoRa.packetRssi() * 2);
    // SNR puede estar en un rango de [20, -148] dBm
    payload[payloadLength++] = uint8_t(148 + LoRa.packetSnr());
    
    transmitting = true;
    txDoneFlag = false;

    tx_begin_ms = millis();

    sendMessage(payload, payloadLength, msgCount);
    Serial.print("Sending packet ");
    Serial.print(msgCount++);
    Serial.print(": ");
    printBinaryPayload(payload, payloadLength);
  }

  // if certain time passes without receiving anything from master
  if (!transmitting && !requestDefaultMode && !defaultMode && waitingToReceive && ((millis() - lastSendTime_ms) > txInterval_ms)){
    Serial.println("Restoring old values...");

    // we set a timer to set to default
    lastRestoreTime_ms = millis();

    requestDefaultMode = true;

    // we set the parameters to the old ones
    restoreOldParameters();

    // resetLora
    resetLora();  

    Serial.print("Old slave config: BW: ");
    Serial.print(bandwidth_kHz[thisNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(thisNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(thisNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(thisNodeConf.txPower);
    Serial.println(" dB\n");
  }


  // if old parameters dont work we set it up to the default
  if (!transmitting && requestDefaultMode && waitingToReceive && !defaultMode && ((millis() - lastRestoreTime_ms) > txInterval_ms)){
    Serial.println("Restoring default values...");

    defaultMode = true;

    // we set the parameters to the default ones
    restoreDefaultParameters();

    // then we update the lora parameters
    resetLora();

    Serial.print("Default slave config: BW: ");
    Serial.print(bandwidth_kHz[thisNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(thisNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(thisNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(thisNodeConf.txPower);
    Serial.println(" dB\n");     

  }

  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs\n");
    
    lastSendTime_ms = tx_begin_ms; 
    
    transmitting = false;

    waitingToReceive = true;
    
    // Reactivamos la recepción de mensajes, que se desactiva
    // en segundo plano mientras se transmite
    LoRa.receive();   
  }


}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) 
{
  while(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);                            // 
  }
  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF)); 
  LoRa.write(payloadLength);              // Añadimos la longitud en bytes del mensaje
  LoRa.write(payload, (size_t)payloadLength); // Añadimos el mensaje/payload 
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a
                                          // finalice su transmisión
}

// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize) 
{
  if (transmitting && !txDoneFlag) txDoneFlag = true;
  
  if (packetSize == 0) return;          // Si no hay mensajes, retornamos

  // we know new lora conf parameters have arrived
  // so we store the actual ones in oldNodeConf
  storeParameters();

  // Leemos los primeros bytes del mensaje
  uint8_t buffer[10];                    // Buffer para almacenar el mensaje
  int recipient = LoRa.read();          // Dirección del destinatario
  uint8_t sender = LoRa.read();         // Dirección del remitente
                                        // msg ID (High Byte first)
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) | 
                            (uint16_t)LoRa.read();
  
  uint8_t incomingLength = LoRa.read(); // Longitud en bytes del mensaje
  
  uint8_t receivedBytes = 0;            // Leemos el mensaje byte a byte
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer)-1))) {            
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  
  if (incomingLength != receivedBytes) {// Verificamos la longitud del mensaje
    Serial.print("Receiving error: declared message length " + String(incomingLength));
    Serial.println(" does not match length " + String(receivedBytes));
    return;                             
  }

  // Verificamos si se trata de un mensaje en broadcast o es un mensaje
  // dirigido específicamente a este dispositivo.
  // Nótese que este mecanismo es complementario al uso de la misma
  // SyncWord y solo tiene sentido si hay más de dos receptores activos
  // compartiendo la misma palabra de sincronización
  if ((recipient & localAddress) != localAddress ) {
    Serial.println("Receiving error: This message is not for me.");
    return;
  }

  // Imprimimos los detalles del mensaje recibido
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Payload length: " + String(incomingLength));
  Serial.print("Payload: ");
  printBinaryPayload(buffer, receivedBytes);
  Serial.print("\nRSSI: " + String(LoRa.packetRssi()));
  Serial.print(" dBm\nSNR: " + String(LoRa.packetSnr()));
  Serial.println(" dB");

  // Actualizamos remoteNodeConf y lo mostramos
  if (receivedBytes == 6) {
    remoteNodeConf.bandwidth_index = buffer[0] >> 4;
    remoteNodeConf.spreadingFactor = 6 + ((buffer[0] & 0x0F) >> 1);
    remoteNodeConf.codingRate = 5 + (buffer[1] >> 6);
    remoteNodeConf.txPower = 2 + ((buffer[1] & 0x3F) >> 1);
    remoteRSSI = -int(buffer[2]) / 2.0f;
    remoteSNR  =  int(buffer[3]) - 148;   
  
    Serial.print("Master config: BW: ");
    Serial.print(bandwidth_kHz[remoteNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(remoteNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(remoteNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(remoteNodeConf.txPower);
    Serial.print(" dBm, RSSI: ");
    Serial.print(remoteRSSI);
    Serial.print(" dBm, SNR: ");
    Serial.print(remoteSNR,1);
    Serial.println(" dB");


    thisNodeConf.bandwidth_index = buffer[4] >> 4;
    thisNodeConf.spreadingFactor = 6 + ((buffer[4] & 0x0F) >> 1);
    thisNodeConf.codingRate = 5 + (buffer[5] >> 6);
    thisNodeConf.txPower = 2 + ((buffer[5] & 0x3F) >> 1); 

    // we set up the new lora conf parameters
    setLoraParameters();

    Serial.print("New slave config: BW: ");
    Serial.print(bandwidth_kHz[thisNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(thisNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(thisNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(thisNodeConf.txPower);
    Serial.println(" dB\n");


//////////////////////////////////////////////
    // We received a packet
    // and we updated lora
    // so we no longer request
    // default mode
    requestDefaultMode = false;

    // node parameters arent at default
    defaultMode = false;

    // We received a packet succesfully
    // so we arent waiting to receive
    waitingToReceive = false;
//////////////////////////////////////////////
    
  }
  else {
    Serial.print("Unexpected payload size: ");
    Serial.print(receivedBytes);
    Serial.println(" bytes\n");
  }

}

void TxFinished()
{
  txDoneFlag = true;
}

void printBinaryPayload(uint8_t * payload, uint8_t payloadLength)
{
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
}

void setLoraParameters(){
  // Configuramos algunos parámetros de la radio

  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index])); 
                                  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);    
  
  LoRa.setCodingRate4(thisNodeConf.codingRate);
                                  
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);   

  Serial.println("Update successful!");


}

void storeParameters(){
  oldNodeConf.bandwidth_index = thisNodeConf.bandwidth_index;
  oldNodeConf.spreadingFactor = thisNodeConf.spreadingFactor;
  oldNodeConf.codingRate = thisNodeConf.codingRate;
  oldNodeConf.txPower = thisNodeConf.txPower;
}

void restoreOldParameters(){
  thisNodeConf.bandwidth_index = oldNodeConf.bandwidth_index;
  thisNodeConf.spreadingFactor = oldNodeConf.spreadingFactor;
  thisNodeConf.codingRate = oldNodeConf.codingRate;
  thisNodeConf.txPower = oldNodeConf.txPower;

}

void restoreDefaultParameters(){
  thisNodeConf.bandwidth_index = defaultNodeConf.bandwidth_index;
  thisNodeConf.spreadingFactor = defaultNodeConf.spreadingFactor;
  thisNodeConf.codingRate = defaultNodeConf.codingRate;
  thisNodeConf.txPower = defaultNodeConf.txPower;  
}

void resetLora(){

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index])); 
                                  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);     
  
  LoRa.setCodingRate4(thisNodeConf.codingRate);         
                                                                    
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
                                                              
  LoRa.setSyncWord(0xB2);                                     
                                  
  LoRa.setPreambleLength(8);      

  LoRa.onReceive(onReceive);

  LoRa.onTxDone(TxFinished);

  LoRa.receive();

  Serial.println("LoRa reset succeeded");


}