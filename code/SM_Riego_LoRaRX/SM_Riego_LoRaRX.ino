// ========================================================================================
// Yafhers Mendoza / Lima - Peru
// ========================================================================================
// SM_Riego_LoRaRX.ino
// Arduino-based LoRa Node Receiver for Soil Moisture and Temperature Monitoring
// ========================================================================================
// Features:
//  - LoRa communication with remote nodes
//  - Bluetooth serial interface for manual/automatic motor control
//  - JSON-like formatted data output for sensor readings
// ========================================================================================          
// Arduino code distributed using a CeCILL-B licence.
//      French version:  https://cecill.info/licences/Licence_CeCILL-B_V1-fr.html
//      English version: https://cecill.info/licences/Licence_CeCILL-B_V1-en.html
// ========================================================================================
// History:
//      Creation by Yafhers Mendoza (yamc1605@gmail.com) : June 25/2025
//      This version corresponds to the creation of LoRa Node Receiver
// ========================================================================================

// Librerias
#include <LoRa_E220.h>
#include <BluetoothSerial.h>

// Frecuencia base nodo LoRa
#define FREQUENCY_868

// ID de nodos LoRa
#define LORA_NODE1_ID  0x00F1

// Pinout LoRa
#define LED_ESP32 2 
#define UART2_TX  17  
#define UART2_RX  16  
#define AUX_PIN   13  
#define M0_PIN    4 
#define M1_PIN    5 
LoRa_E220 e220ttl(UART2_RX, UART2_TX, &Serial2, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600);

// Objetos
BluetoothSerial SerialBT;

// Estructura para el manejo de datos
typedef struct
{
  uint16_t idNodo;
  uint16_t readingId;
  float    humedad;
  float    temp;
}Message_t;

// Estructura para el manejo de datos del motor
typedef struct
{
  uint16_t  idNodo;
  uint16_t  readingId;
  uint8_t   command;
}MessageMotor_t;

// Variables
char  str_json[100];
uint16_t idMessage = 0;
Message_t recvMsg  = {0};
MessageMotor_t motMsg = {0};
float humedadSuelo = 0.0;
float temperaturaSuelo = 0.0;
String bluetoothCommand = "";

// Prototipos de Funciones
void LoRa_Init(void);

// Prototipos de Tasks
void processDataBluetooth(void* pvParameters);
void sendDataBluetooth(void* pvParameters);
void receiveLoRa(void* pvParameters);

// Void Setup
void setup() 
{
  // Configuración Serial
  Serial.begin(115200);

  // Configuracion de nodo LoRa
  LoRa_Init();

  // Configuración Bluetooth
  if (SerialBT.begin("ESP32_Bluetooth_Node")) {
    Serial.println("ESP-BT: Bluetooth inciado correctamente ✅");
  } 
  else {
    Serial.println("ESP-BT: Hubo un error al iniciar el bluetooth ⛔");
  }

  // Configuración de Pines
  pinMode(LED_ESP32, OUTPUT);
  digitalWrite(LED_ESP32, LOW);

  // Configuración de Tasks
  xTaskCreate(processDataBluetooth, "Process Bluetooth", 4096, NULL, 3, NULL);
  xTaskCreate(sendDataBluetooth, "Send Data Bluetooth", 2048, NULL, 3, NULL);
  xTaskCreate(receiveLoRa, "Receive LoRa", 4096, NULL, 3, NULL);

  // Elimino el loop
  vTaskDelete(NULL);
}

// Task: Enviar datos de sensores mediante Bluetooth
void sendDataBluetooth(void* pvParameters)
{ 
  String esp32Buffer = "";

  // Loop del Task
  while(1) {
    // Formar la cadena con los datos a enviar
    esp32Buffer = String(humedadSuelo, 2) + "," + String(temperaturaSuelo, 2); 

    // Enviar datos a Bluetooth
    Serial.println("ESP-BT: " + esp32Buffer);
    SerialBT.println(esp32Buffer);

    // Espera entre envio de datos
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// Task: Procesar datos recibidos mediante Bluetooth
void processDataBluetooth(void* pvParameters)
{
  // Variables locales
  String bluetoothBuffer = "";
  bool flagMode = 0;

  // Loop del Task
  while(1) {
    // Leer datos del Bluetooth
    while (SerialBT.available()) {
      // Leer byte recibido mediante Bluetooth
      char c = SerialBT.read();

      // Cuando detecta el caracter de terminación acabo la recepción
      if(c == '\n') {
        bluetoothCommand = bluetoothBuffer;
        bluetoothBuffer = "";
      }
      else {
        bluetoothBuffer += c;
      }
    }

    // Procesar comando recibido del Bluetooth
    if (bluetoothCommand.length() > 0)
    {

      // Activar modo manual
      if (bluetoothCommand.indexOf("MAN") >= 0) {
        flagMode = true;
        Serial.println("ESP-BT: Modo Manual activado 📢🟣");
        motMsg.command = 1;
      }
      // Activar modo automatico
      else if (bluetoothCommand.indexOf("AUT") >= 0) {
        flagMode = false;
        digitalWrite(LED_ESP32, LOW);
        Serial.println("ESP-BT: Modo Automatico activado 📢🔵");
        motMsg.command = 2;
      }

      // Modo Manual
      if (flagMode == 1)
      {
        // Encender la bomba de agua
        if (bluetoothCommand.indexOf("ON") >= 0) {
          digitalWrite(LED_ESP32, HIGH);
          Serial.println("ESP-BT: Motor Encendido ✔");
          motMsg.command = 3; 
        }
        // Apagar la bomba de agua
        else if (bluetoothCommand.indexOf("OFF") >= 0) {
          digitalWrite(LED_ESP32, LOW);
          Serial.println("ESP-BT: Motor Apagado ❌");
          motMsg.command = 4;
        }
      }

      // Envio de mensaje al nodo LoRa
      motMsg.readingId++;
      ResponseStatus rsTx = e220ttl.sendFixedMessage(0x00, 0x4A, 65, (const uint8_t *)&motMsg, sizeof(motMsg));

      // Verificar el envio del mensaje
      Serial.printf("ESP32-LoRa: Comando a enviar: %d \r\n", motMsg.command);
      if (rsTx.code == 1) Serial.printf("ESP-LoRa: Envio de mensaje al gateway Ok ✔ \r\n");
      else Serial.println("ESP-LoRa: Envio de mensaje al gateway fallido ❌ \r\n");
      
      // Limpiar el comando
      bluetoothCommand = "";
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task: Receive Lora
void receiveLoRa(void* pvParameters)
{
  while(1)
  {
    // Si llega un mensaje al modulo LoRa
    if (e220ttl.available() > 1) {
      Serial.println("Ha llegado un mensaje LoRa 🔥");
      ResponseStructContainer rscRx = e220ttl.receiveMessage(sizeof(recvMsg));
      memcpy((uint8_t *)&recvMsg, (uint8_t *)rscRx.data, sizeof(recvMsg));

      // Si recibimos un mensaje con codigo valido
      if (rscRx.status.code == 1) {
        // Decodificar los datos recibidos
        idMessage = recvMsg.idNodo;

        // Si el mensaje tiene un ID correcto
        if (idMessage == LORA_NODE1_ID) {
          // Formatear los datos en JSON
          sprintf(str_json, "HUM: %.2f, TEM: %.2f", recvMsg.humedad, recvMsg.temp);
          Serial.printf("ESP-LoRa: %s \r\n", str_json);

          // Almacenar datos recibidos
          humedadSuelo = recvMsg.humedad;
          temperaturaSuelo = recvMsg.temp;
        } 
      }
      else {
        Serial.println("Error al recibir datos");
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Void Loop (sin usar)
void loop()
{

}

// Funcion para incializar el modulo LoRa
void LoRa_Init(void)
{
  // Variable para obtener configuracion del modulo LoRa
  ResponseStructContainer rc;

  // Inicia todos los pines y UART
	e220ttl.begin();

  // Leer configuracion actual
  rc = e220ttl.getConfiguration();

  if(rc.status.code == 1)
  {
    Serial.println("ESP_LoRa: Lectura de configuracion exitosa 🐋");
  }
  else {
    Serial.println("ESP_LoRa: Error en la lectura de configuracion 💀");
  }

  // Es importante obtener el puntero de configuración antes de cualquier otra operación
  Configuration config = *(Configuration*) rc.data;

  // Configuración de dirección del módulo LoRa
  config.ADDH = 0x00; 
  config.ADDL = 0x1A;  

  // Communication channel
  config.CHAN = 65; 

  config.SPED.uartBaudRate = UART_BPS_9600;                             // Serial baud rate
  config.SPED.airDataRate  = AIR_DATA_RATE_000_24;                      // Air baud rate 2.4Kbps
  config.SPED.uartParity   = MODE_00_8N1;                               // Parity bit

  config.OPTION.subPacketSetting = SPS_200_00;                          // Packet size
  config.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;         // Need to send special command
  config.OPTION.transmissionPower = POWER_22;                           // Device power

  config.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;                  // Enable RSSI info
  config.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;   //FT_TRANSPARENT_TRANSMISSION

  config.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;                    // Check interference
  config.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;                    // WOR timing

  //Establecer la configuracion del modulo LoRa
  e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
  rc.close();

  Serial.println("ESP_LoRa: Configuracion completa 🌟");
}