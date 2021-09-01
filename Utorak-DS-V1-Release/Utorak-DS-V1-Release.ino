/*
   Project: Utorak Detection System 
   Version: V1
   Status:  Released
   Date:    08/08/2020
   Client:  Utorak
   
   Developer: Luciano Aguerre
   Contact: luchoaguerre@gmail.com
   Powered by NIMJI Electronics
   
  
   Update:
   Modification pending.

C:\Users\laguerre\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\3.0.0/esptool.exe --chip esp32 --port COM7 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xe000 C:\Users\laguerre\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.6/tools/partitions/boot_app0.bin 0x1000 C:\Users\laguerre\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.6/tools/sdk/bin/bootloader_qio_80m.bin 0x10000 C:\Users\laguerre\AppData\Local\Temp\arduino_build_508062/Utorak-DS-V1-Release.ino.bin 0x8000 C:\Users\laguerre\AppData\Local\Temp\arduino_build_508062/Utorak-DS-V1-Release.ino.partitions.bin 


*/

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <rom/rtc.h>
#include <driver/adc.h>
#include <WiFi.h>
#include <time.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <SPIFFS.h>
#include <EEPROM.h>
#include "soc/rtc_cntl_reg.h"

// HARDWARE DEFINITIONS
//--------------------------------------------------------------------------------
#define LED_RED   23 
#define LED_BLUE  22 
#define ACT_BZ    4  
#define ACT_VI    32 
#define ALIM_ST   33 
#define ALIM_CH   26 
#define ALIM_BAT  25
#define ALIM_USB  27
#define SW_PRIN   16 
#define SW_SEC    0  

// SYSTEM DEFINES 
//-----------------------------------------------------------------------------------
#define N_SENSOR  50                    //Nro de BLE_RSSI_DISP_arrayositivos para el array de nodos


// TIMER VARIABLES
//---------------------------------------------------------------------------------
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// SWITCHES INTERRUPTS VARIABLES
//-----------------------------------------------------------------------------------
portMUX_TYPE SW_PRINmux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE SW_SECmux = portMUX_INITIALIZER_UNLOCKED;

// SYSTEM VARIABLES
//---------------------------------------------------------------------------------
 
const char*     NETWORK_NAME = "UTORAK_DEVICE";

byte            SYSTEM_STATE = 0;
const long      SYSTEM_BASE_TIME = 10000;         // microsegundos de la base de tiempo - para que interrumpa cada 10 mS
const long      SYSTEM_BASE_TIME_SEC_MUL = 1000000 / SYSTEM_BASE_TIME;
const long      SYSTEM_BASE_TIME_MIN_MUL = 60000000 / SYSTEM_BASE_TIME;

bool            ALIM_USB_state = 0;

const long      BATTERY_time_measure = 1 * SYSTEM_BASE_TIME_MIN_MUL;        // cada 1 minuto
const long      BATTERY_ALERT_time = 1 * SYSTEM_BASE_TIME_MIN_MUL;          // cada 1 minutos
byte            BATTERY_estado = 0;
volatile long   BATTERY_ticks = BATTERY_time_measure;     // Se BLE_RSSI_DISP_arrayara cada 1 minuto.
byte            BATTERY_valor = 0;
float           BATTERY_tension = 0;
#define         BATTERY_LEVEL_LOW 3.5                //Valores para la detección de batería baja y muerta - hay que ver bien estos valores después
#define         BATTERY_LEVEL_DEAD 3.3                   
bool            BATTERY_CHG_COMPLETE = false;             // Este flag es necesario, dado que hay un problema con el TP4056, cuando acercas la mano activa el pin STANBY, debe ser por la alta impedancia.
volatile long   BATTERY_ALERT_ticks = BATTERY_ALERT_time; // Se BLE_RSSI_DISP_arrayara cada 5 minutos

byte            SW_PRINCIPAL_state = 0;
volatile long   SW_PRINCIPAL_ticks = 0;
byte            SW_PRINCIPAL_valor = 0;
const long      SW_PRINCIPAL_largo = 1 * SYSTEM_BASE_TIME_SEC_MUL;     

bool            SW_PRIN_ACT = 0;            //Variable para detectar activación de SW_Principal en la interrupción
bool            SW_SEC_ACT = 0;             //Variable para detectar activación de SW_Secundario en la interrupción

byte            SW_SECUNDARIO_state = 0;
volatile long   SW_SECUNDARIO_ticks = 0;
byte            SW_SECUNDARIO_valor = 0;
const long      SW_SECUNDARIO_largo = 1 * SYSTEM_BASE_TIME_SEC_MUL; 

const long      LED_BLUE_HEARBEAT_freq = 3 * SYSTEM_BASE_TIME_SEC_MUL;
volatile long   LED_BLUE_HEARTBEAT_ticks = LED_BLUE_HEARBEAT_freq;

volatile long   PROX_IND_RESET = 100;

const long      WATCHDOG_PERIOD = 10 * SYSTEM_BASE_TIME_SEC_MUL;
volatile long   WATCHDOG_TIME = WATCHDOG_PERIOD;

const long      DEEP_SLEEP_MULTIPLIER = 1000000;
const long      DEEP_SLEEP_PERIOD     = 1 * DEEP_SLEEP_MULTIPLIER;

int             BLE_RSSI = 0;
int             BLE_RSSI_KALMAN = 0;

const long      BLE_EVAL_PERIOD = 10 * SYSTEM_BASE_TIME_SEC_MUL;
volatile long   BLE_EVAL_ticks  = BLE_EVAL_PERIOD;

const long      BLE_DETECTION_PERIOD_1 = 1 * SYSTEM_BASE_TIME_SEC_MUL;
volatile long   BLE_DETECTION_TIMER_1  = BLE_DETECTION_PERIOD_1;
const long      BLE_DETECTION_PERIOD_2 = 5 * SYSTEM_BASE_TIME_SEC_MUL;
volatile long   BLE_DETECTION_TIMER_2  = BLE_DETECTION_PERIOD_2;
bool            BLE_DETECTION_SNOOZE_estado = 0;
const long      BLE_DETECTION_SNOOZE_period = 5 * SYSTEM_BASE_TIME_MIN_MUL;
volatile long   BLE_DETECTION_SNOOZE_ticks  = BLE_DETECTION_SNOOZE_period;
int             BLE_DETECTION_LIMIT_array[] = {200,180,160,140};  //Arranca con la deteccion en 200cm
byte            BLE_DETECTION_LIMIT_index = 0; 
bool            BLE_DETECTION_near = false;                       //Variable para detectar si está cerca 

char            MAC_BLE[18]     = {0};    //Base MAC must be a unicast MAC (least significant bit of first byte must be zero). length: 6 bytes.
char            MAC_WIFI[18]    = {0};   //Base MAC must be a unicast MAC (least significant bit of first byte must be zero). length: 6 bytes.
String          MAC_CONVERT;   //Base MAC must be a unicast MAC (least significant bit of first byte must be zero). length: 6 bytes.

String          BLE_RSSI_DISP_array[N_SENSOR];      //Para guardar las MAC o parte de la MAC de los dipositivos que van apareciendo
int             BLE_RSSI_array[N_SENSOR];          
float           BLE_RSSI_KALMAN_array[N_SENSOR];   
int             BLE_RSSI_NEAR_array[N_SENSOR];     
int             BLE_RSSI_DISTANCE_array[N_SENSOR];
volatile long   BLE_RSSI_TICKS_array[N_SENSOR];
const long      BLE_RSSI_TICKS_RST = 1 * SYSTEM_BASE_TIME_SEC_MUL;
int             j = 0;                              //Inicializa índice de vector de BLE_RSSI_DISP_arrayositivos (o nodos)

String          WIFI_RSSI_DISP_array[N_SENSOR];      //Para guardar las MAC o parte de la MAC de los dipositivos que van apareciendo
int             WIFI_RSSI_array[N_SENSOR];
int             i = 0;

const long      WIFI_DETECTION_PERIOD = 10 * SYSTEM_BASE_TIME_SEC_MUL;
volatile long   WIFI_DETECTION_ticks  = WIFI_DETECTION_PERIOD;

const char*     WIFI_ssid = "Speedy-0FB5CD_plus";
const char*     WIFI_password =  "0355866334";
const char*     WIFI_SCAN_SET_ssid = "UTORAK_DEV";
const char*     WIFI_SCAN_SET_pass = "kdijrefr7393hd93243";

struct tm       timeinfo;
time_t          now;

const char*     SPIFFS_Archivo = "/records";

#define         EEPROM_SIZE 2
//---------------------------
// EEPROM Memory map
// adr 0        BLE_DETECTION_LIMIT_index
// adr 1        SYSTEM_STATE      

//---------------------------------------------------------------------------------
// KALMAN PARAMETERS
const float     KALMAN_q_slow = 0.01;
const float     KALMAN_q_fast = 0.1; 
const float     KALMAN_R = 50;                      // noise covariance calculation, s^2
float           KALMAN_q = KALMAN_q_fast;           // system noise or ESP32 position noise.  Q cerca a 1 sigue mas las variaciones. Q cerca de 0 es mas duro.
float           KALMAN_U_est = 90;                  // Initial estimated state RSSI
float           KALMAN_P = 50;                      // Initial error covariance
float           KALMAN_K = 0;                       // Initial Kalman gain
float           KALMAN_U_est_array[N_SENSOR];              
float           KALMAN_P_array[N_SENSOR];                  
float           KALMAN_K_array[N_SENSOR];                  
const long      KALMAN_TIMER_PERIOD = 30 * SYSTEM_BASE_TIME_SEC_MUL;
volatile long   KALMAN_TIMER_ticks  = KALMAN_TIMER_PERIOD;
//--------------------------------------------------------------------------------


// BLE VARIABLES
//----------------------------------------------------------------------------------
static          BLEAddress *pServerAddress;         //Puntero para manejo de función BLE
int             BLE_scanTime = 1;                   //Intervalo de escaneo BLE completo (en segundos)
BLEScan*        pBLEScan;                                  //Puntero para escaneo BLE
BLEScanResults  foundDevices;                        //Para capturar lo devuelto por el método BLE_scan->start (cantidad de dispositivos encontrados durante el escaneo)
BLEAdvertising  *pAdvertising;                       //Puntero para notificación BLE

//-----------------------------------------------------------------------------
float KALMAN_FILTER (int U , int a)
{
  float KALMAN_result;
   
  KALMAN_K_array[a] = KALMAN_P_array[a] / (KALMAN_P_array[a] + KALMAN_R);                             // Kalman gain calculation
  KALMAN_U_est_array[a] = (KALMAN_K_array[a] * U) + ((1-KALMAN_K_array[a]) * KALMAN_U_est_array[a]);  // Measurement stimation
  KALMAN_P_array[a] = ((1-KALMAN_K_array[a]) * KALMAN_P_array[a]) + KALMAN_q;                         // Update the error covariance
  KALMAN_result = KALMAN_U_est_array[a];
            
  return KALMAN_result;
}
void KALMAN_TIMER (void)
{
  if (!KALMAN_TIMER_ticks) 
  { 
    KALMAN_TIMER_ticks = KALMAN_TIMER_PERIOD;
    KALMAN_q = KALMAN_q_fast;
  }
}
//--------------------------------------------------------------------------------------
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks        //Callback escaneo BLE
{
 
    void onResult(BLEAdvertisedDevice advertisedDevice)       //Función que se ejecuta cuando se encuentra un disp BLE                     
    {
      if (digitalRead(SW_PRIN))               // si se apreto el boton
      //if(SW_PRIN_ACT)                         // si se apreto el boton
        advertisedDevice.getScan()->stop();   // que salga del scaneo. 
     
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());   //Obtiene la dirección del disp que notifica
      
      if (advertisedDevice.getName() == NETWORK_NAME)      //Si pertenece a la red...
      {    
        //-----------------------------------------------------------------------------------------------
          j = 0;                                                //Inicializa índice de vector de dispositivos (o nodos)
          MAC_CONVERT = advertisedDevice.getAddress().toString().c_str();
          MAC_CONVERT.toUpperCase();
          BLE_MAC_TRANSFORM();
          while(!(BLE_RSSI_DISP_array[j]).equals(MAC_CONVERT))  //Mientras que no encuentra el dispositivo actual escaneado en el vector:
          {
              if (((BLE_RSSI_DISP_array[j]).equals("")))                                             //Si el string de esta posición está vacío
                BLE_RSSI_DISP_array[j] = MAC_CONVERT;                 //guarda la dirección en esta posición del array           
              else                                                                    //De lo contrario, si esta posición no está vacía 
              {
                if (j < N_SENSOR)                                                     //Si el puntero es menor al tamaño total del array
                  j++;                                                                //Incrementa puntero a siguiente elemento del array
              }
          }
        //------------------------------------------------------------------------------------------------------------------------ 
        BLE_RSSI_array[j] = -(advertisedDevice.getRSSI());                             //Captura el valor del RSSI
        BLE_RSSI_KALMAN_array[j] = KALMAN_FILTER(BLE_RSSI_array[j] , j);  
        BLE_RSSI_DISTANCE_array[j] = BLE_DISTANCE_ESTIMATION(BLE_RSSI_KALMAN_array[j]);   
        BLE_RSSI_TICKS_array[j] = BLE_RSSI_TICKS_RST;
        //-----------------------------------------------------------------------------------------------------------------
        
         //  rutina para setear si esta near
         //---------------------------------
        if (BLE_RSSI_DISTANCE_array[j] <= BLE_DETECTION_LIMIT_array[BLE_DETECTION_LIMIT_index])
        {
          BLE_RSSI_NEAR_array [j] = 1;
          BLE_EVAL_NEAR();
          BLE_DETECTION_FLASH();

          KALMAN_q = KALMAN_q_slow;           // modifica los valores del filtro
          KALMAN_TIMER_ticks = KALMAN_TIMER_PERIOD;
          //advertisedDevice.getScan()->stop();   // que salga del scaneo.   
        }
        //if (BLE_RSSI_DISTANCE_array[j] > BLE_DETECTION_LIMIT_array[BLE_DETECTION_LIMIT_index])
        else
        {
          BLE_RSSI_NEAR_array [j] = 0;
        } 
        advertisedDevice.getScan()->stop();                                       //Si son iguales detiene el escaneo (esto se hace para comenzar rápidamente otro escaneo y poder tomar
      }
    }       
};          //Fin de Callback

//--------------------------------------------------------------------------------------
// SERVICIO DE INTERRUPCION DEL TIMER
//--------------------------------------------------------------------------------------
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  // Critical Code begins here
  //---------------------------------------------------------------------------
  //if (SYSTEM_STATE && !BLE_DETECTION_TIMER_1)  digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
  if (BATTERY_ticks) BATTERY_ticks--;
  if (BATTERY_ALERT_ticks) BATTERY_ALERT_ticks--;
  if (LED_BLUE_HEARTBEAT_ticks) LED_BLUE_HEARTBEAT_ticks--;
  if (BLE_DETECTION_SNOOZE_ticks) BLE_DETECTION_SNOOZE_ticks--;
  if (BLE_DETECTION_TIMER_1) BLE_DETECTION_TIMER_1--;
  if (BLE_DETECTION_TIMER_2) BLE_DETECTION_TIMER_2--;
  if (BLE_EVAL_ticks) BLE_EVAL_ticks--;
  if (KALMAN_TIMER_ticks) KALMAN_TIMER_ticks--;
  if (WIFI_DETECTION_ticks) WIFI_DETECTION_ticks--;
  if (WATCHDOG_TIME) 
    WATCHDOG_TIME--;
  else  
    ESP.restart();                            //Si el Watchdog llegó a 0 ejecuta un SW_RESET
  for (int RSSI_index = 0; RSSI_index < N_SENSOR; RSSI_index++)
      if (BLE_RSSI_TICKS_array[RSSI_index]) BLE_RSSI_TICKS_array[RSSI_index]--;
  
  if ((digitalRead(SW_PRIN)) && (SW_PRINCIPAL_ticks < 60000)) SW_PRINCIPAL_ticks++;
  if ((!digitalRead(SW_SEC)) && (SW_SECUNDARIO_ticks < 60000)) SW_SECUNDARIO_ticks++;
  //----------------------------------------------------------------------------
  portEXIT_CRITICAL_ISR(&timerMux);
}

//--------------------------------------------------------------------------------------
// SERVICIO DE INTERRUPCION DEL PULSADOR PRINCIPAL
//--------------------------------------------------------------------------------------
void IRAM_ATTR SW_PRIN_ISR()
{
  portENTER_CRITICAL_ISR(&SW_PRINmux);
  if (!SW_PRIN_ACT) 
  {
    SW_PRIN_ACT = 1;
    if (SYSTEM_STATE)                                         //Solo si SYSTEM_STATE=1 (está escaneando)
    {
      pBLEScan->stop();                                       //que salga del scaneo. 
      pBLEScan->clearResults();                               //y limpie la memoria
    }  
  }  
  portEXIT_CRITICAL_ISR(&SW_PRINmux);
}
//--------------------------------------------------------------------------------------
// SERVICIO DE INTERRUPCION DEL PULSADOR SECUNDARIO
//--------------------------------------------------------------------------------------
void IRAM_ATTR SW_SEC_ISR()
{
  portENTER_CRITICAL_ISR(&SW_SECmux);
  if (!SW_SEC_ACT) 
  {
    SW_SEC_ACT = 1;
    if (SYSTEM_STATE)                                         //Solo si SYSTEM_STATE=1 (está escaneando)
    {
      pBLEScan->stop();                                       //que salga del scaneo. 
      pBLEScan->clearResults();                               //y limpie la memoria
    }  
  }   
  portEXIT_CRITICAL_ISR(&SW_SECmux);
}

//-----------------------------------------------------------------------------------------
//  SECCION DE RUTINAS PARA EL MANEJO DEL BLE
//-----------------------------------------------------------------------------------------
void BLE_MAC_TRANSFORM (void)
{
  char MAC_AUX[] = "XX:XX:XX:XX:XX:XX";
  String LAST_BYTE  = "XX";

  MAC_CONVERT.toCharArray(MAC_AUX, sizeof(MAC_AUX));

}
void BLE_CFG(void)
{
  //  Serial.println("Scanning...");
  pBLEScan = BLEDevice::getScan();          //Crea nuevo escaneo
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(),true);    //Setea la Callback
  pBLEScan->setActiveScan(true);  //Usa escaneo activo
  pBLEScan->setInterval(10);      //Intervalos de 10 mS para cada dispositivo (para escanear más rápido)
  pBLEScan->setWindow(9);         //Define ventana de escaneo

  // Configuración de Potencia de BLE y notificación
  BLEDevice::init(NETWORK_NAME);                 //Inicia este dispositivo con el nombre 
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT,ESP_PWR_LVL_P9);    // Set default connection power to max
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);       // Set advertising power to max
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);      // Set scan power to max 

  pAdvertising = BLEDevice::getAdvertising();           
  pAdvertising->setScanResponse(true);                                
  pAdvertising->setMinPreferred(0x0);   
}
//----------------------------------------------------------------------------------------
void BLE_DETECTION(void)
{
  foundDevices = pBLEScan->start(BLE_scanTime, false);    //Arranca el escaneo durante 1 segundo (o hasta que se detenga en la Callback) y devuelve la cantidad de dispositivos encontrados
  pBLEScan->clearResults();                               //Borra los resultados del buffer  de BLEScan para liberar memoria

  if(BLE_DETECTION_near)                                                            //Si ve dispositivos cerca
    LED_BLUE_HEARTBEAT_ticks = LED_BLUE_HEARBEAT_freq;                //Reinicia contador para bloquear el modo latido - solo cuando no ve dispositivos empieza a decrementar el contador 
  
  //-----------------------------------------------------------------------------------------------------------------------------
  for (int BLE_RSSI_index2 = 0; BLE_RSSI_index2 < N_SENSOR;BLE_RSSI_index2++)
  {
    if ((!BLE_RSSI_TICKS_array[BLE_RSSI_index2]) && (BLE_RSSI_DISP_array[BLE_RSSI_index2] != ""))
    {
      KALMAN_U_est_array[BLE_RSSI_index2] = KALMAN_U_est;      
      KALMAN_P_array[BLE_RSSI_index2] = KALMAN_P;          
      KALMAN_K_array[BLE_RSSI_index2] = KALMAN_K;                  //Inicializa los vectores de las variables de KALMAN
      // Borra del vector los dispositivos que dejaron de ser detectados
      BLE_RSSI_DISP_array[BLE_RSSI_index2] = "";      
      BLE_RSSI_array[BLE_RSSI_index2] = 0;           
      BLE_RSSI_KALMAN_array[BLE_RSSI_index2] = 0;    
      BLE_RSSI_NEAR_array[BLE_RSSI_index2] = 0;      
      BLE_RSSI_DISTANCE_array[BLE_RSSI_index2] = 0;
    }  
  }
}
//-------------------------------------------------------------------------------------
void BLE_SAVE_DATA(int j)
{
           Serial.print("DETECCION: ");
           NTP_Get_LocalTime();
           Serial.print(&timeinfo, "%d.%b.%Y, %H:%M:%S");
           Serial.print("MAC: ");
           Serial.println(BLE_RSSI_DISP_array[j]);
}
//-------------------------------------------------------------------------------------
void BLE_EVAL_NEAR (void)
{
    int BLE_EVAL_result = 0;
    
    for (int BLE_cont = 0; BLE_cont < N_SENSOR;BLE_cont++)
      BLE_EVAL_result = BLE_EVAL_result + BLE_RSSI_NEAR_array [BLE_cont];

    if (BLE_EVAL_result)
      BLE_DETECTION_near = true;
    else
      BLE_DETECTION_near = false;
}
//---------------------------------------------------------------------------------------------------------------
int BLE_DISTANCE_ESTIMATION (float x)
{
// valores con el equipo colgado en el cuerpo, con TX_power 9
float a = 0.2654;
float b = 31.253;
float c = 965.86;

  float result = 0;
  
  result = a * (x * x) - b * (x) + c;
  return result;
}
//---------------------------------------------------------------------------------------------------------------
void BLE_SERIAL_PRINT(void)
{
    int indice = 0;
    while(!(BLE_RSSI_DISP_array[indice]).equals(""))  //Mientras que no encuentra el dispositivo actual escaneado en el vector:
    {
        Serial.print(BLE_RSSI_DISP_array[indice]);
        Serial.print("RSSI: ");
        Serial.print(BLE_RSSI_KALMAN_array[indice]);
        Serial.print("  d: ");
        Serial.print(BLE_RSSI_DISTANCE_array[indice]);
        Serial.print("  ");    
        //Serial.print(WIFI_RSSI_DISP_array[indice]);
        //Serial.print(": WIFI ");
        //Serial.println(WIFI_RSSI_array[indice]);
        indice++;
    }
    Serial.println("  ");
}
//------------------------------------------------------------------------------------------------------------
void BLE_CLEAR_VECTORS(void)
{
  for(j = 0; j<N_SENSOR ; j++)
  {
    KALMAN_U_est_array[j] = KALMAN_U_est;      
    KALMAN_P_array[j] = KALMAN_P;          
    KALMAN_K_array[j] = KALMAN_K;                  //Inicializa los vectores de las variables de KALMAN

    BLE_RSSI_DISP_array[j] = "";      //Para guardar las MAC o parte de la MAC de los dipositivos que van apareciendo
    BLE_RSSI_array[j] = 0;           //Para filtro KALMAN de los BLE_RSSI_DISP_arrayositivos
    BLE_RSSI_KALMAN_array[j] = 0;    //Para filtro KALMAN de los BLE_RSSI_DISP_arrayositivos
    BLE_RSSI_NEAR_array[j] = 0;      //Para filtro KALMAN de los BLE_RSSI_DISP_arrayositivos
    BLE_RSSI_DISTANCE_array[j] = 0;
       
    WIFI_RSSI_DISP_array[j] = "";      //Para guardar las MAC o parte de la MAC de los dipositivos que van apareciendo
    WIFI_RSSI_array[j] = 0;
  }
}
//-------------------------------------------------------------------------------------------------------------
void BLE_DETECTION_SNOOZE (void)
{
  if (!BLE_DETECTION_SNOOZE_ticks)
    BLE_DETECTION_SNOOZE_estado = 0;
}
//-------------------------------------------------------------------------------------------------------------
void BLE_DETECTION_FLASH (void)
{
          if ((BLE_DETECTION_near) && (!BLE_DETECTION_TIMER_2))                       //si detecta un equipo cerca 
          {
            digitalWrite(LED_BLUE,LOW);
            digitalWrite(LED_RED,HIGH);
            if (!BLE_DETECTION_SNOOZE_estado)
            {
              digitalWrite(ACT_VI,HIGH);
              digitalWrite(ACT_BZ,HIGH);
            }
            BLE_DETECTION_TIMER_1 = BLE_DETECTION_PERIOD_1;
            BLE_DETECTION_TIMER_2 = BLE_DETECTION_PERIOD_2;
            LED_BLUE_HEARTBEAT_ticks = LED_BLUE_HEARBEAT_freq;
          }
          if(!BLE_DETECTION_TIMER_1)                //Si contador de 2 segundos llegó a cero significa que no detectó cercanía
          {
            digitalWrite(LED_RED,LOW);
            digitalWrite(ACT_VI,LOW);
            digitalWrite(ACT_BZ,LOW);
          }
}
//-------------------------------------------------------------------------------------------------------------
void WIFI_SCAN_SET (void)
{
//    WIFI_POWER_19_5dBm = 78,// 19.5dBm
//    WIFI_POWER_19dBm = 76,// 19dBm
//    WIFI_POWER_18_5dBm = 74,// 18.5dBm
//    WIFI_POWER_17dBm = 68,// 17dBm
//    WIFI_POWER_15dBm = 60,// 15dBm
//    WIFI_POWER_13dBm = 52,// 13dBm
//    WIFI_POWER_11dBm = 44,// 11dBm
//    WIFI_POWER_8_5dBm = 34,// 8.5dBm
//    WIFI_POWER_7dBm = 28,// 7dBm
//    WIFI_POWER_5dBm = 20,// 5dBm
//    WIFI_POWER_2dBm = 8,// 2dBm
//    WIFI_POWER_MINUS_1dBm = -4// -1dBm
  
  WiFi.setTxPower(WIFI_POWER_5dBm);
  //Serial.println(WiFi.getTxPower());
  WiFi.softAP(WIFI_SCAN_SET_ssid, WIFI_SCAN_SET_pass);
}

void WIFI_DETECTION (void)
{
  if (!WIFI_DETECTION_ticks)
  {
    WIFI_DETECTION_ticks = WIFI_DETECTION_PERIOD;
    WIFI_SCAN();
  }
}
//--------------------------------------------------------------------------------------------------------------
void WIFI_SCAN(void)
{
  
      int WIFI_SCAN_nodes = WiFi.scanNetworks();
      if (WIFI_SCAN_nodes == 0) 
      {
            Serial.println("no networks found");
      } 
      else 
      {
            for (int WIFI_SCAN_CONT = 0; WIFI_SCAN_CONT < WIFI_SCAN_nodes; WIFI_SCAN_CONT++) 
            {
                // Print SSID and RSSI for each network found
                if (WiFi.SSID(WIFI_SCAN_CONT) == WIFI_SCAN_SET_ssid)
                {
                    i = 0;
                    while((WIFI_RSSI_DISP_array[i]) != (WiFi.BSSIDstr(WIFI_SCAN_CONT)))  //Mientras que no encuentra el dispositivo actual escaneado en el vector:
                    {
                        if (((WIFI_RSSI_DISP_array[i]).equals("")))                      //Si el string de esta posición está vacío
                          WIFI_RSSI_DISP_array[i] = WiFi.BSSIDstr(WIFI_SCAN_CONT);       //guarda la dirección en esta posición del array           
                        else                                                             //De lo contrario, si esta posición no está vacía 
                        {
                          if (i < N_SENSOR)                                              //Si el puntero es menor al tamaño total del array
                            i++;                                                         //Incrementa puntero a siguiente elemento del array
                        }
                    }
                    //-------------------------------------------------------------------------------------------------- 
                    WIFI_RSSI_array[i] = -WiFi.RSSI(WIFI_SCAN_CONT);                 //Captura el valor del RSSI
                    delay(5);
                }
            }
     }
}
//-----------------------------------------------------------------------------------------
//  RUTINA PARA CHEQUEAR SI SE PULSO EL BOTON PRINCIPAL
//-----------------------------------------------------------------------------------------
void SWITCH_PRINCIPAL_CHECK(void)
{
   if (digitalRead(SW_PRIN))    //chequea si hay pulsación del sw principal
   {
      SW_PRINCIPAL_ticks = 0;         //Si la hay, Resetea los ticks      
      while(digitalRead(SW_PRIN) && (SW_PRINCIPAL_ticks < SW_PRINCIPAL_largo) )     //mientras sigue pulsado se queda acá
        WATCHDOG_TIME = WATCHDOG_PERIOD;      //Recarga el watchdog para que el equipo no se resetee

      if(SW_PRINCIPAL_ticks >= (SW_PRINCIPAL_largo-1))       //Cuando sale chequea los ticks: Si superan los correspondientes a la pulsación larga
        SW_PRINCIPAL_valor = 2;                         // Pulsacion larga 
      else              //Si no superan los de la pulsación larga
      {
        if (SW_PRINCIPAL_ticks > 5)      //Pero superan los del debounce
           SW_PRINCIPAL_valor = 1;  // Pulsacion corta                
        else
           SW_PRINCIPAL_valor = 0;
      }
   }
   SW_PRIN_ACT = 0;      //Al final de todo, resetea flag de Interrupción  
}
//-----------------------------------------------------------------------------------------
//  RUTINA PARA CHEQUEAR SI SE PULSO EL BOTON SECUNDARIO
//-----------------------------------------------------------------------------------------
void SWITCH_SECUNDARIO_CHECK(void)
{
   if (!digitalRead(SW_SEC))    //chequea si hay pulsación del sw sec
   {
      SW_SECUNDARIO_ticks = 0;         //Si la hay, Resetea los ticks      
      while(!digitalRead(SW_SEC) && (SW_SECUNDARIO_ticks < SW_SECUNDARIO_largo))     //mientras sigue pulsado se queda acá
        WATCHDOG_TIME = WATCHDOG_PERIOD;      //Recarga el watchdog para que el equipo no se resetee

      if(SW_SECUNDARIO_ticks >= (SW_SECUNDARIO_largo-1))             //Si los ticks superan los del debounce
        SW_SECUNDARIO_valor = 2;              // Pulsacion valida 
      else
      {
        if (SW_SECUNDARIO_ticks > 5)
          SW_SECUNDARIO_valor = 1;
        else
          SW_SECUNDARIO_valor = 0;
      }
   }
   SW_SEC_ACT = 0;      //Al final de todo, resetea flag de Interrupción  
}

//-------------------------------------------------------------------------------------------------------------                 
void SWITCH_PRINCIPAL_ACCIONS (void)
{
  if ((SW_PRINCIPAL_valor == 1) && SYSTEM_STATE == 1)
  {
    SW_PRINCIPAL_valor = 0;
    SW_PRINCIPAL_state = 0;
    VIB(200);
    switch (BLE_DETECTION_SNOOZE_estado) {
      case 0: // Snooze ON
          BLE_DETECTION_SNOOZE_estado = 1;
          BLE_DETECTION_SNOOZE_ticks  = BLE_DETECTION_SNOOZE_period;
        break;

      case 1: // Snooze OFF
          BLE_DETECTION_SNOOZE_estado = 0;
          BLE_DETECTION_SNOOZE_ticks  = BLE_DETECTION_SNOOZE_period;
        break;
    }
  }
  if (SW_PRINCIPAL_valor == 2)
  {
    SW_PRINCIPAL_valor = 0;
    SW_PRINCIPAL_state = 0;
    
    switch (SYSTEM_STATE) {
      case 0: // Encender
        digitalWrite(LED_BLUE,HIGH);
        digitalWrite(LED_RED,LOW);
        VIB(600);
        digitalWrite(LED_BLUE,LOW);
        SYSTEM_STATE = 1;
        EEPROM.write(1, SYSTEM_STATE);
        EEPROM.commit();
        BLEDevice::startAdvertising(); // Arranca el BLE
        Serial.print("arranca el BLE");
        LED_BLUE_HEARTBEAT_ticks = LED_BLUE_HEARBEAT_freq;                //Reinicia contador para lanzar modo latido 
        break;

      case 1: // Apagar
        digitalWrite(LED_BLUE,HIGH);
        digitalWrite(LED_RED,LOW);
        VIB(600);
        digitalWrite(LED_BLUE,LOW);
        SYSTEM_STATE = 0;
        EEPROM.write(1, SYSTEM_STATE);
        EEPROM.commit();
        pBLEScan->stop(); // apaga el scaneo
        pAdvertising->stop();  // Apaga el BLE
        Serial.print("apago el BLE");
        break;
    }
  }
}
//-----------------------------------------------------------------------------------------
void SWITCH_SECUNDARIO_ACCIONS (void)
{
  if (SW_SECUNDARIO_valor == 1)
  {
    SW_SECUNDARIO_valor = 0;
     
    BLE_DETECTION_LIMIT_index++;  
    if (BLE_DETECTION_LIMIT_index > sizeof(BLE_DETECTION_LIMIT_array)) BLE_DETECTION_LIMIT_index = 0;
    EEPROM.write(0, BLE_DETECTION_LIMIT_index);
    EEPROM.commit();
    VIB(300);
  }
  
  if (SW_SECUNDARIO_valor == 2)
  {
    SW_SECUNDARIO_valor = 0;
    BLE_DETECTION_LIMIT_index = 0;  
    VIB(500);
  }
}
//-----------------------------------------------------------------------------------------
//  RUTINA PARA CHEQUEAR EL ESTADO DE LA BATERIA
//-----------------------------------------------------------------------------------------
void BATTERY_CHECK(void)  // Funciona OK
{
  int BATTERY_MEASURE = 0;
  if (!BATTERY_ticks)
  {
    BATTERY_ticks = BATTERY_time_measure;
    adc2_get_raw( ADC2_CHANNEL_8 , ADC_WIDTH_12Bit, &BATTERY_MEASURE);  //Read analog
    BATTERY_tension = (float)BATTERY_MEASURE * 6.7/4095;  // (2 x 3.7V /4095)
    BATTERY_valor  =  0; // bateria ok
    if (BATTERY_tension < BATTERY_LEVEL_LOW) 
      BATTERY_valor = 1; //bateria baja
      
    if (BATTERY_tension < BATTERY_LEVEL_DEAD)
      BATTERY_valor = 2; //bateria muerta
  }
}
//-----------------------------------------------------------------------------------------
//  RUTINA PARA CHEQUEAR SI LA BATERIA TERMINO DE CARGARSE
//-----------------------------------------------------------------------------------------
void BATTERY_CHARGE_CHECK (void)
{
  /*
  STNDBY (Pin6): When the battery Charge Termination, the pin is pulled low by an internal switch, otherwise pin is in high impedance state.
  CHRG   (Pin7): When the battery is being charged, the pin is pulled low by an internal switch, otherwise pin is in high impedance state
*/
  if ((digitalRead(ALIM_ST)) && (!BATTERY_CHG_COMPLETE))  // chequea si ya se dejo de cargar la bateria
  {
    digitalWrite(LED_RED,HIGH);                 // si esta en carga, se enciende el led rojo
    digitalWrite(LED_BLUE,LOW);                 //Apaga led azul  
  }
  else
  {
    digitalWrite(LED_RED,LOW); // si ya esta cargada se enciende el led azul
    digitalWrite(LED_BLUE,HIGH);
    BATTERY_CHG_COMPLETE = true;
  }
}
//-----------------------------------------------------------------------------------------
//  RUTINA PARA INDICAR QUE HAY BATERIA BAJA, PARPADEA EL VIBRADOR Y EL BUZZER CADA 1 MINUTOS
//-----------------------------------------------------------------------------------------
void BATTERY_ALERT(void)   
{
  if (!BATTERY_ALERT_ticks)
  {
    BATTERY_ALERT_ticks = BATTERY_ALERT_time;

    switch (BATTERY_valor) {
      case 0: 
           // Todo OK
           break;
      case 1:  
            // Bateria Baja, debe Flashear
            digitalWrite(ACT_BZ,HIGH);
            digitalWrite(ACT_VI,HIGH);
            delay(200);
            digitalWrite(ACT_BZ,LOW);
            digitalWrite(ACT_VI,LOW);
            delay(100);
            digitalWrite(ACT_BZ,HIGH);
            digitalWrite(ACT_VI,HIGH);
            delay(200);
            digitalWrite(ACT_BZ,LOW);
            digitalWrite(ACT_VI,LOW);
          break;
      case 2:
            // Bateria Muerta
            digitalWrite(LED_BLUE,HIGH);
            digitalWrite(LED_RED,LOW);
            VIB(600);
            digitalWrite(LED_BLUE,LOW);
            SYSTEM_STATE = 0;
            EEPROM.write(1, SYSTEM_STATE);
            EEPROM.commit();
            pBLEScan->stop(); // apaga el scaneo
            pAdvertising->stop();  // Apaga el BLE
            esp_sleep_enable_timer_wakeup(DEEP_SLEEP_PERIOD);
            esp_deep_sleep_start();                         //Manda al equipo a dormir por 10 minutos 
            break;
      default:
            break;
    }
  }
}

//-------------------------------------------------------------------------------------------------------------
void NTP_WIFI_CFG (void)
{
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_ssid, WIFI_password);
  while (WiFi.status() != WL_CONNECTED)
  {
    WATCHDOG_TIME = WATCHDOG_PERIOD;
  }

  //-------- NTP Time sync    ----------------------------------------------
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
  delay(1000);
    // wait for time to be set
    time(&now);
    // Set timezone to Argentina Standard Time
    setenv("TZ", "WART4WARST", 1);
    tzset();
    localtime_r(&now, &timeinfo);
  //----------------------------------------------
  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
}
//-------------------------------------------------------------------------------------------------------------
void NTP_Get_LocalTime(void)
{
  time(&now);
  localtime_r(&now, &timeinfo);
  //Serial.println(&timeinfo, "%d.%b.%Y, %H:%M:%S");
}
//-------------------------------------------------------------------------------------
void HARDWARE_CFG (void)
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(ACT_BZ, OUTPUT);
  pinMode(ACT_VI, OUTPUT);
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_BLUE,LOW);
  digitalWrite(ACT_BZ,LOW);
  digitalWrite(ACT_VI,LOW);
  
  pinMode(ALIM_ST, INPUT_PULLUP); 
  pinMode(ALIM_CH, INPUT_PULLUP); 
  pinMode(ALIM_BAT, INPUT);
  pinMode(ALIM_USB, INPUT);
  pinMode(SW_PRIN, INPUT); 
  pinMode(SW_SEC, INPUT_PULLUP); 
  
  adc2_config_channel_atten(ADC2_CHANNEL_8,ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
  attachInterrupt(digitalPinToInterrupt(SW_PRIN), SW_PRIN_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SW_SEC), SW_SEC_ISR, LOW);
}

void TIMER_CFG(void)
{
  timer = timerBegin(0, 80, true); // 80 es la frecuencia del oscilador
  timerAttachInterrupt(timer, &onTimer, true);
  // second parameter is the base time in us
  timerAlarmWrite(timer, SYSTEM_BASE_TIME, true);  // 10.000us -> 10ms
  timerAlarmEnable(timer);
}

void GENERAL_CFG(void)
{
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  
  for(j = 0; j<N_SENSOR ; j++)
  {
    KALMAN_U_est_array[j] = KALMAN_U_est;      
    KALMAN_P_array[j] = KALMAN_P;          
    KALMAN_K_array[j] = KALMAN_K;                  //Inicializa los vectores de las variables de KALMAN
  }
}

//-----------------------------------------------------------
void READ_MACS (void)
{
  unsigned char MAC[6];  
  
  // aca deberia de leer la mac de este dispositivo y guardarla
  esp_read_mac(MAC,ESP_MAC_BT); //es porque la MAC para BLE
  sprintf(MAC_BLE, "%02X:%02X:%02X:%02X:%02X:%02X", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
  Serial.println(MAC_BLE);
  esp_read_mac(MAC,ESP_MAC_WIFI_SOFTAP);
  sprintf(MAC_WIFI, "%02X:%02X:%02X:%02X:%02X:%02X", MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]);
  Serial.println(MAC_WIFI);
}

//-----------------------------------------------------------------------------------------
//  Rutina para el control del vibrador al enchufar o desenchufar la alimentacion
//-----------------------------------------------------------------------------------------
void ALIM_PLUG_CHECK(void)
{
  if (digitalRead(ALIM_USB) && (!ALIM_USB_state))  // si es distinto del estado anterior es porque hubo un cambio, entonces debe vibrar.
  {
    // funciones de apagado y stand by
    pBLEScan->stop();
    pAdvertising->stop();       //En este momento detiene escaneo y notificación BLE
    digitalWrite(LED_RED,LOW);    //Apago los dos LEDs
    digitalWrite(LED_BLUE,LOW);
    ALIM_USB_state = true;
    SYSTEM_STATE = 0;
    EEPROM.write(1, SYSTEM_STATE);
    EEPROM.commit();
    VIB(300);
  }
  if (!digitalRead(ALIM_USB) && (ALIM_USB_state)) // si esta desconectado, que pase al siguiente estado
  {
    ALIM_USB_state = false;
    BATTERY_CHG_COMPLETE = false;
    digitalWrite(LED_RED,LOW);    //Apago los dos LEDs
    digitalWrite(LED_BLUE,LOW);
    VIB(300);
  }
}
//----------------------------------------------------------------------------------------------------------
void LED_BLUE_HEARTBEAT(void)
{
  if (digitalRead(SW_PRIN))              // si se apreto el boton principal sale
        return; 

  if (!digitalRead(SW_SEC))               // si se apreto el boton secundario sale
        return; 
          
  if ((!LED_BLUE_HEARTBEAT_ticks) && (SYSTEM_STATE == 1))
  {
    LED_BLUE_HEARTBEAT_ticks = LED_BLUE_HEARBEAT_freq;
    //Serial.println("Modo heartbeat");
    LED_BLUE_PULSE();
  }
}

void LED_BLUE_PULSE(void)
{
//unsigned int    LED_PULSE_brillo[] = {0,1,2,3,5,6,8,10,12,14,16,19,22,25,29,33,38,43,51,65,80,97,100};
unsigned int    LED_PULSE_brillo[] = {0,1,2,3,5,6,8,10,12,14,16,19,22,25,29,33,38,43,51,65,65,65,65}; // para bajar el brillo maximo
unsigned int    LED_PULSE_repeat = 100;
const int       LED_PULSE_velocidad = 200;
byte            LED_PULSE_index = 0;
byte            LED_PULSE_delta = 0;


  for (LED_PULSE_index = 0; LED_PULSE_index < 23; LED_PULSE_index++){
    for (LED_PULSE_repeat = 0; LED_PULSE_repeat < LED_PULSE_velocidad; LED_PULSE_repeat++){
      LED_PULSE_delta = LED_PULSE_brillo[LED_PULSE_index];
      digitalWrite(LED_BLUE,HIGH);
      delayMicroseconds (LED_PULSE_delta);
      if (LED_PULSE_delta == 100)
      {
        delayMicroseconds (100-LED_PULSE_delta);    
      }
      else
      {
        digitalWrite(LED_BLUE,LOW);
        delayMicroseconds (100-LED_PULSE_delta);
      }
      if (digitalRead(SW_PRIN))               // si se apreto el boton
        return;

      if (!digitalRead(SW_SEC))               // si se apreto el boton secundario sale
        return; 
    }
   }
   for (LED_PULSE_index = 23; LED_PULSE_index > 0; LED_PULSE_index--){
    for (LED_PULSE_repeat = 0; LED_PULSE_repeat < LED_PULSE_velocidad; LED_PULSE_repeat++){
      LED_PULSE_delta = LED_PULSE_brillo[LED_PULSE_index];
      digitalWrite(LED_BLUE,HIGH);
      delayMicroseconds (LED_PULSE_delta);
      digitalWrite(LED_BLUE,LOW);
      delayMicroseconds (100-LED_PULSE_delta);
      if (digitalRead(SW_PRIN))               // si se apreto el boton
        return; 
        
      if (!digitalRead(SW_SEC))               // si se apreto el boton secundario sale
        return;         
    }
   }
}
//-----------------------------------------------------------------------------------------
//  RUTINA PARA INDICAR QUE HUBO UNA ACTIVACION, PARPADEA TODO 3 VECES
//  
//-----------------------------------------------------------------------------------------
void VIB(int VIB_milis)
{
  digitalWrite(ACT_VI,HIGH);
  delay(VIB_milis);
  digitalWrite(ACT_VI,LOW);
}
// ------------------------------------------------------------------------------------------------------------
//                  SPIFFS ROUTINES
//-------------------------------------------------------------------------------------------------------------
void SPIFFS_ReadFile(const char * path)
{
    File file = SPIFFS.open(path);
    if(!file){
        Serial.println("- failed to open file for reading");
        return;
    }
    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void SPIFFS_WriteFile(const char * path, const char * message)
{
    File file = SPIFFS.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void SPIFFS_AppendFile(const char * path, const char * message)
{
    File file = SPIFFS.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void SPIFFS_DeleteFile(const char * path)
{
    if(SPIFFS.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}
/*
 Calculo de capacidad
 MAC Adress XX:XX:XX:XX:XX:XX 17 bytes
 Fecha: 10.jun.2020,20:12:50  20 bytes
 Separador ,                  1  byte
 Retorno de carro             2  bytes
 ------------------------------------
                        total 40 bytes 

1Mb = 1.048.576    / 40 bytes = 26.214 tramas / 5 dias = 5.242 interacciones por dia.
2Mb = 2.097.152    / 40 bytes = 52.428 tramsa / 5 dias = 10.485 interacciones por dia.
                        
 */

// -----------------------------------------------------------------------------------------------------------
//                  RESET REASON ANALSYS
// -----------------------------------------------------------------------------------------------------------
void RESET_REASON_GET(void)
{
   if ((rtc_get_reset_reason(1) == 3) || (rtc_get_reset_reason(1) == 12))   // si hay SW Reset  
   {
        SYSTEM_STATE = EEPROM.read(1);  
        if (SYSTEM_STATE)
        {                      
            BLEDevice::startAdvertising();    // Arranca el BLE 
            LED_BLUE_HEARTBEAT_ticks = LED_BLUE_HEARBEAT_freq;                //Reinicia contador para lanzar modo latido 
            digitalWrite(LED_RED,LOW);
            digitalWrite(LED_BLUE,LOW);
            BLE_DETECTION_LIMIT_index = EEPROM.read(0);
            if (BLE_DETECTION_LIMIT_index > sizeof(BLE_DETECTION_LIMIT_array)) BLE_DETECTION_LIMIT_index = 0;
        }
   }
   //---------------------------------------------------------------------------------------------------------------
   if (rtc_get_reset_reason(1) == 5)                   // si se reseta por deepsleep
   {
        if (BATTERY_valor == 2)
        {
          if (!ALIM_USB_state)
          {
              esp_sleep_enable_timer_wakeup(DEEP_SLEEP_PERIOD);
              esp_deep_sleep_start(); 
          }
        }
   }
}
// ------------------------------------------------------------------------------------------------------------
//                  SETUP ROUTINE
//-------------------------------------------------------------------------------------------------------------
void setup() 
{
    // disable brownout detector to maximize battery life
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  HARDWARE_CFG();  
  GENERAL_CFG();    
  TIMER_CFG(); 
  //NTP_WIFI_CFG();
  BLE_CFG();           
  //WIFI_SCAN_SET();

  RESET_REASON_GET();
}

// ------------------------------------------------------------------------------------------------------------
//                  MAIN LOOP ROUTINE
//-------------------------------------------------------------------------------------------------------------
void loop() 
{
  ALIM_PLUG_CHECK();    //chequea que no se haya enchufado la alimentacion, deberia ser como una interrupcion por eso va afuera del switch
  if (!ALIM_USB_state) // Si NO se está cargando. ejecuta el progama normal.
  {
    WATCHDOG_TIME = WATCHDOG_PERIOD;
    SWITCH_PRINCIPAL_CHECK();
    SWITCH_PRINCIPAL_ACCIONS();
    switch (SYSTEM_STATE) 
    {
        case 0:
                esp_sleep_enable_timer_wakeup(DEEP_SLEEP_PERIOD);
                esp_deep_sleep_start(); 
                break;
        case 1:
                // funcionando modo normal
                if (!SW_PRIN_ACT)    //aca hay un flag de interrupcion
                {
                  BLE_DETECTION();                //hace un escaneo del BLE 
                  //WIFI_DETECTION();        
                }
                //BLE_EVAL();
                KALMAN_TIMER();                 // hace un ajuste automatico de la ganancia del filtro.
                BLE_SERIAL_PRINT();
                BLE_EVAL_NEAR();
                BLE_DETECTION_FLASH();
                SWITCH_SECUNDARIO_CHECK();
                SWITCH_SECUNDARIO_ACCIONS();  
                BATTERY_CHECK();
                BATTERY_ALERT();
                BLE_DETECTION_SNOOZE();
                if ((!SW_PRIN_ACT)&&(!SW_SEC_ACT)) 
                    LED_BLUE_HEARTBEAT();       //Solo si no se pulsó el botón principal llama a la función latido
                break;
    }
  }
  else                // Si se está cargando chequea solo el estado de carga de la bateria.
  {
    BATTERY_CHARGE_CHECK();
    WATCHDOG_TIME = WATCHDOG_PERIOD;
  }
}
