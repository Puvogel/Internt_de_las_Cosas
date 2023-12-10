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

#define TX_LAPSE_MS          10000

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0xBB;     // Dirección de este dispositivo
uint8_t destination = 0xC4;            // Dirección de destino, 0xFF es la dirección de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

bool cleanEnoughSignal = false;
bool strongEnoughSignal = false;
bool instantSend = true;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

double SNRRequiredForModulation[13] = { 0.0,    0.0,     0.0,
                                        0.0,    0.0,     0.0,
                                       -5.0,   -7.5,   -10.0,
                                      -12.5,  -15.0,   -17.5,  -20.0};


double SNRRequiredForModulationCR[9] = { 0.0,    0.0,     0.0,
                                         0.0,    0.0,    -5.0,
                                       -10.0,  -15.5,   -20.0};
                                      

LoRaConfig_t thisNodeConf   = { 4, 12, 8, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};

LoRaConfig_t newNodeConf = { 4, 12, 8, 2};
LoRaConfig_t oldNodeConf = { 4, 12, 8, 2};
LoRaConfig_t defaultNodeConf = { 4, 12, 8, 2};

int remoteRSSI = 0;
float remoteSNR = 0;

bool waitingToReceive = false;
bool requestDefaultMode = false;

bool defaultMode = false;

uint8_t iterator = 0;

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
  static uint32_t lastTxCompleted_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;
  static uint32_t lastRestoreTime_ms = 0;
      
  if (!transmitting && !waitingToReceive && ((millis() - lastTxCompleted_ms) > txInterval_ms) || instantSend) {

    instantSend = false;

    Serial.println("Computing new parameters...");
    computeNewParameters();


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

    payload[payloadLength]    = (newNodeConf.bandwidth_index << 4);
    payload[payloadLength++] |= ((newNodeConf.spreadingFactor - 6) << 1);
    payload[payloadLength]    = ((newNodeConf.codingRate - 5) << 6);
    payload[payloadLength++] |= ((newNodeConf.txPower - 2) << 1);    
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(payload, payloadLength, msgCount);
    Serial.print("Sending packet ");
    Serial.print(msgCount++);
    Serial.print(": ");
    printBinaryPayload(payload, payloadLength);
  }
  
  // if certain time passes without receiving anything from slave
  if (!transmitting && !requestDefaultMode && waitingToReceive && ((millis() - lastTxCompleted_ms) > txInterval_ms)){
    Serial.println("Restoring old values...");
    

    requestDefaultMode = true;

    // we dont wait to receive any longer
    // because we are going to send a data
    // with new configutation
    waitingToReceive = false;

    // restore parameters to old ones
    restoreOldParameters();

    // set lora up with this parameters
    resetLora();

    Serial.print("Old master config: BW: ");
    Serial.print(bandwidth_kHz[thisNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(thisNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(thisNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(thisNodeConf.txPower);
    Serial.println(" dB\n");

    Serial.println("We wait till slave restores to old\n");
    delay(16500);

    // we set a timer to set to default
    lastRestoreTime_ms = millis();

  }

  // if after restoring old parameters master aint receiving data
  // we restore it to default values
  if (!transmitting && requestDefaultMode && waitingToReceive && !defaultMode && ((millis() - lastRestoreTime_ms) > 15000)){
    Serial.println("Restoring default values...");
    
    // timer to know when was the last restore time
    lastRestoreTime_ms = millis();

    // now we are at default values
    defaultMode = true;

    // also msg count starts to be 0 again
    msgCount = 0;

    // we dont wait to receive any longer
    // because we are going to send a data
    // with new configutation
    waitingToReceive = false;

    iterator = 0;

    // we set the parameters to the default ones
    restoreDefaultParameters();

    // then we update the lora parameters
    resetLora();

    Serial.print("Default master config: BW: ");
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
    
    lastTxCompleted_ms = millis(); 
    
    transmitting = false;

    waitingToReceive = true;
    
    storeParameters();

    // we change the actual parameters
    // to the new ones
    updateThisParameters();

    // we update this parameters to Lora
    setLoraParameters();

    Serial.print("Master changed to: BW: ");
    Serial.print(bandwidth_kHz[thisNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(thisNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(thisNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(thisNodeConf.txPower);
    Serial.println(" dB\n");

    // as we changed the Parameters default
    // is no longer the Lora setup
    defaultMode = false;
    
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

  

  // Leemos los primeros bytes del mensaje
  uint8_t buffer[10];                   // Buffer para almacenar el mensaje
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
  if (receivedBytes == 4) {
    remoteNodeConf.bandwidth_index = buffer[0] >> 4;
    remoteNodeConf.spreadingFactor = 6 + ((buffer[0] & 0x0F) >> 1);
    remoteNodeConf.codingRate = 5 + (buffer[1] >> 6);
    remoteNodeConf.txPower = 2 + ((buffer[1] & 0x3F) >> 1);
    remoteRSSI = -int(buffer[2]) / 2.0f;
    remoteSNR  =  int(buffer[3]) - 148;
  
    Serial.print("Slave config: BW: ");
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

    Serial.println("The new parameters were effective\n");

    
//////////////////////////////////////////////
    // We received a packet
    // so new config works
    // no need for requesting
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



void computeNewParameters(){

  computeBandwidth_index();
  computeSpreadingFactor();
  computeCodingRate();
  computeTxPower();

}

void computeBandwidth_index(){

  if (-80 < LoRa.packetRssi() && newNodeConf.bandwidth_index < 9){
    newNodeConf.bandwidth_index+=1;
  }
  else if (newNodeConf.txPower == 20 && -80 > LoRa.packetRssi()){
    newNodeConf.bandwidth_index-=1;
  }

}

void computeSpreadingFactor(){
  
  // So there is a table which sais how much SNR is able to support
  // any of the spreading factor modes for example for a spreading
  // factor of 10 you are able to go down to -15dB, for a spreading
  // factor of 9 you are able to go down to -12.5sB, so what I do
  // in this "if else" code is that, if we are at SF10 for example
  // and the SNR is equual or superior to -12.5, we are able to 
  // change the SF to 9 but if the SNR is less than -15dB we would
  // change to SF11.

  if (SNRRequiredForModulation[newNodeConf.spreadingFactor-1] <= LoRa.packetSnr() && newNodeConf.spreadingFactor > 7){
    newNodeConf.spreadingFactor -= 1;
  } 
  else if (SNRRequiredForModulation[newNodeConf.spreadingFactor] > LoRa.packetSnr() && newNodeConf.spreadingFactor < 12){
    newNodeConf.spreadingFactor += 1;
  }
  
}

void computeCodingRate(){
  if (SNRRequiredForModulationCR[newNodeConf.codingRate-1] <= LoRa.packetSnr() && newNodeConf.codingRate > 5){
    newNodeConf.codingRate -= 1;
  } 
  else if (SNRRequiredForModulationCR[newNodeConf.codingRate] > LoRa.packetSnr() && newNodeConf.codingRate < 8){
    newNodeConf.codingRate += 1;
  }

}

void computeTxPower(){
  if (LoRa.packetRssi() <= -80 && LoRa.packetRssi() > -90 && newNodeConf.txPower < 20){
    newNodeConf.txPower+=1;
  }
  else if (LoRa.packetRssi() <= -90 && newNodeConf.txPower < 19){
    newNodeConf.txPower+=2;
  }
  else if (LoRa.packetRssi() <= -90 && newNodeConf.txPower == 19){
    newNodeConf.txPower+=1;
  }  
  else if (LoRa.packetRssi() > -67 && newNodeConf.txPower > 2){
    newNodeConf.txPower-=1;
  }
  else if (LoRa.packetRssi() > -30 && newNodeConf.txPower > 3){
    newNodeConf.txPower-=2;
  }  
}

void updateThisParameters(){
  thisNodeConf.bandwidth_index = newNodeConf.bandwidth_index;
  thisNodeConf.spreadingFactor = newNodeConf.spreadingFactor;
  thisNodeConf.codingRate = newNodeConf.codingRate;
  thisNodeConf.txPower = newNodeConf.txPower;

  if (thisNodeConf.spreadingFactor == 6){

  }
  else {

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
  if (oldNodeConf.bandwidth_index != newNodeConf.bandwidth_index){
    oldNodeConf.bandwidth_index = thisNodeConf.bandwidth_index;
  }

  if (oldNodeConf.spreadingFactor != newNodeConf.spreadingFactor){
    oldNodeConf.spreadingFactor = thisNodeConf.spreadingFactor;
  }
    
  if (oldNodeConf.codingRate != newNodeConf.codingRate){
    oldNodeConf.codingRate = thisNodeConf.codingRate;
  }

  if (oldNodeConf.txPower != newNodeConf.txPower){
    oldNodeConf.txPower = thisNodeConf.txPower;
  }
}

void restoreOldParameters(){
  thisNodeConf.bandwidth_index = oldNodeConf.bandwidth_index;
  thisNodeConf.spreadingFactor = oldNodeConf.spreadingFactor;
  thisNodeConf.codingRate = oldNodeConf.codingRate;
  thisNodeConf.txPower = oldNodeConf.txPower;

  newNodeConf.bandwidth_index = oldNodeConf.bandwidth_index;
  newNodeConf.spreadingFactor = oldNodeConf.spreadingFactor;
  newNodeConf.codingRate = oldNodeConf.codingRate;
  newNodeConf.txPower = oldNodeConf.txPower;
}

void restoreDefaultParameters(){
  thisNodeConf.bandwidth_index = defaultNodeConf.bandwidth_index;
  thisNodeConf.spreadingFactor = defaultNodeConf.spreadingFactor;
  thisNodeConf.codingRate = defaultNodeConf.codingRate;
  thisNodeConf.txPower = defaultNodeConf.txPower;  

  newNodeConf.bandwidth_index = defaultNodeConf.bandwidth_index;
  newNodeConf.spreadingFactor = defaultNodeConf.spreadingFactor;
  newNodeConf.codingRate = defaultNodeConf.codingRate;
  newNodeConf.txPower = defaultNodeConf.txPower;    
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
