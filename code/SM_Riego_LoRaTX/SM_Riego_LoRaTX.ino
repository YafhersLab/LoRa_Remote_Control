// ========================================================================================
// Yafhers Mendoza / Lima - Peru
// ========================================================================================
// SM_Riego_LoRaTX.ino
// Arduino-based LoRa Node Transmitter for Soil Moisture and Temperature Monitoring
// ========================================================================================
// Features:
//  - LoRa communication with remote gateway nodes
//  - Capacitive soil moisture sensor readings
//  - DS18B20 temperature sensor integration
//  - Manual/Automatic mode control via LoRa commands
//  - JSON-like formatted data output for sensor readings
// ========================================================================================          
// Arduino code distributed using a CeCILL-B licence.
//      French version:  https://cecill.info/licences/Licence_CeCILL-B_V1-fr.html
//      English version: https://cecill.info/licences/Licence_CeCILL-B_V1-en.html
// ========================================================================================
// History:
//      Creation by Yafhers Mendoza (yamc1605@gmail.com) : February 22/2026
//      Initial version of LoRa Node Transmitter for Lab-STICC soil monitoring project
// ========================================================================================

// Librerias
#include <LoRa_E220.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pines de sensores y actuadores
#define LED_ESP32     2 
#define HUM_PIN       35
#define TEM_PIN       5

// Frecuencia base nodo LoRa
#define FREQUENCY_868

// ID de nodos LoRa
#define LORA_NODE1_ID  0x00F1

// Pines del modulo LoRa
#define UART2_TX  17
#define UART2_RX  16 
#define AUX_PIN   13 
#define M0_PIN    4 
#define M1_PIN    18 
LoRa_E220 e220ttl(UART2_RX, UART2_TX, &Serial2, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600);

// Objetos
OneWire myWire(TEM_PIN);
DallasTemperature DS18B20(&myWire);

// Estructura para el manejo de datos LoRa TX
typedef struct
{
  uint16_t idNodo;
  uint16_t readingId;
  float    humedad;
  float    temp;
}Message_t;

// Estructura para el manejo de datos LoRa RX
typedef struct
{
  uint16_t  idNodo;
  uint16_t  readingId;
  uint8_t   command;
}MessageMotor_t;

// Variables de aplicacion
char  str_json[100];
uint16_t idMessage = 0;
Message_t sendMsg = {0};
MessageMotor_t motMsg = {0};
float humedadSuelo = 0.0;
float humedadSuelo_past = 0.0;
float temperaturaSuelo = 0.0;
float temperaturaSuelo_past = 0.0;
volatile bool flagMode = 0;
volatile bool flagMotor = 0;
volatile uint8_t command = 0;

// Prototipos de Funciones
void LoRa_Init(void);

// Prototipos de Tasks
void processDataBluetooth(void* pvParameters);
void readSensors(void* pvParameters);

// Void Setup
void setup() 
{
  // Configuración Serial
  Serial.begin(115200);

  // Configuración de Pines
  pinMode(LED_ESP32, OUTPUT);
  digitalWrite(LED_ESP32, LOW);
  flagMotor = 0;

  // Configuración de nodo LoRa
  LoRa_Init();
  sendMsg.idNodo = 0x00F1;

  // Configuración del sensor de Humedad Capacitivo
  analogReadResolution(10);

  // Configuración del sensor de Temperatura DS18B20
  DS18B20.begin();

  // Configuración de Tasks
  xTaskCreate(processDataBluetooth, "Process Bluetooth", 4096, NULL, 3, NULL);
  xTaskCreate(readSensors, "Read Sensors", 4096, NULL, 3, NULL);

  // Elimino el loop
  vTaskDelete(NULL);
}

// Task: Procesar datos recibidos mediante Bluetooth
void processDataBluetooth(void* pvParameters)
{
  // Variables locales
  String bluetoothBuffer = "";

  // Loop del Task
  while(1) {

    // Si llega un mensaje al modulo LoRa
    if (e220ttl.available() > 1) {
      //Serial.println("Ha llegado un mensaje LoRa 🔥");
      ResponseStructContainer rscRx = e220ttl.receiveMessage(sizeof(motMsg));
      memcpy((uint8_t *)&motMsg, (uint8_t *)rscRx.data, sizeof(motMsg));
      
      // Si recibimos un mensaje con codigo valido
      if (rscRx.status.code == 1) {
        // Decodificar los datos recibidos
        idMessage = motMsg.idNodo;

        // Imprimir los datos
        sprintf(str_json, "CMD: %d", motMsg.command);
        Serial.printf("ESP-LoRa: %s \r\n", str_json);

        // Almacenar datos recibidos
        command = motMsg.command;

        // Procesar comando recibido del LoRa
        // Activar modo manual
        if (command == 1)
        {
          flagMode = true;
          Serial.println("ESP-BT: Modo Manual activado 📢🟣");
        }
        // Activar modo automatico
        else if (command == 2)
        {
          flagMode = false;
          digitalWrite(LED_ESP32, LOW);
          Serial.println("ESP-BT: Modo Automatico activado 📢🔵");
          flagMotor = 0;
        }

        // Modo Manual
        if (flagMode == true)
        {
          // Encender la bomba de agua
          if (command == 3) {

            digitalWrite(LED_ESP32, HIGH);
            Serial.println("ESP-BT: Motor Encendido ✔");
            flagMotor = 1;
          }
          // Apagar la bomba de agua
          else if (command == 4) {
            digitalWrite(LED_ESP32, LOW);
            Serial.println("ESP-BT: Motor Apagado ❌");
            flagMotor = 0;
          }
        }
        
      }
      else {
        Serial.println("Error al recibir datos");
      }
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// Task: Lectura de sensores de humedad y temperatura y envio de datos
void readSensors(void* pvParameters)
{
  while(1)
  {
    // Lectura del sensor de humedad
    humedadSuelo = map(analogRead(35), 100, 820, 100, 0);
    sendMsg.humedad = humedadSuelo;

    // Lectura del sensor de temperatura
    DS18B20.requestTemperatures();
    temperaturaSuelo = DS18B20.getTempCByIndex(0);
    if(temperaturaSuelo > 0) temperaturaSuelo_past = temperaturaSuelo;
    sendMsg.temp = temperaturaSuelo_past;

    /*
    // Si estas en modo automatico
    if (flagMode == 0) {
      // Cuando la maceta se encuentra seca encender motor
      if (humedadSuelo > 400) {
        digitalWrite(LED_ESP32, HIGH);
        flagMotor = 1;
      } 
      else {
        digitalWrite(LED_ESP32, LOW);
        flagMotor = 0;
      }
    } 
    */

    // Envio de mensaje al nodo LoRa
    sendMsg.readingId++;
    ResponseStatus rsTx = e220ttl.sendFixedMessage(0x00, 0x1A, 65, (const uint8_t *)&sendMsg, sizeof(sendMsg));

    //Imprimir datos
    Serial.printf("ESP32: Datos Leídos: %.2f, %.2f \r\n", sendMsg.humedad, sendMsg.temp);

    // Verificar el envio del mensaje
    if (rsTx.code == 1) Serial.printf("ESP-LoRa: Envio de mensaje al gateway Ok ✔ \r\n");
    else Serial.println("ESP-LoRa: Envio de mensaje al gateway fallido ❌ \r\n");

    // Espera entre lecturas
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Void Loop (sin usar)
void loop()
{

}

// Funcion: Inicializar el modulo LoRa
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
    //Serial.println("ESP_LoRa: Lectura de configuracion exitosa 🐋");
  }
  else {
    //Serial.println("ESP_LoRa: Error en la lectura de configuracion 💀");
  }

  // Es importante obtener el puntero de configuración antes de cualquier otra operación
  Configuration config = *(Configuration*) rc.data;

  // Configuración de dirección del módulo LoRa
  config.ADDH = 0x00; 
  config.ADDL = 0x4A;  

  // Communication channel = 850.125 + 65 = 915.125 Mhz
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