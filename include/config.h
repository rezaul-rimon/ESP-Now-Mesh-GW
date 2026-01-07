
// This code is designed to run on an ESP32 device with a SIM7600 GSM module.
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
//======================================//

// === Project Configuration === //
// #define USE_SD_CARD
#define USE_FastLED
//======================================//

//Libraries required for GSM, MQTT, and ESP-NOW functionality
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <esp_task_wdt.h>
#include <FastLED.h>
#include <Preferences.h>
#include <Update.h>

#define CONFIG_TASK_WDT_DEBUG 1

Preferences preferences;

// Debugging macros
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTF(x)  if (DEBUG_MODE) { Serial.printf(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }
//==========================================//

//Device Configuration
#define CHANGE_DEICE_ID 0

#if CHANGE_DEICE_ID
    #define WORK_PACKAGE "1180"
    #define GW_TYPE "03"
    #define FIRMWARE_UPDATE_DATE "260107" 
    #define DEVICE_SERIAL "0009"
#endif

const char* DEVICE_ID;
//========================================//


//Timers for publishing dat/a and heartbeat
unsigned long lastDataPublishTime = 0;
const unsigned long dataPublishInterval = 5 * 60 * 1000;

unsigned long lastHBPublishTime = 0;
const unsigned long hbPublishInterval = 2 * 60 * 1000;

bool ledState = false;
//========================================//

//FastLED library for controlling LEDs
#ifdef USE_FastLED
    #define LED_PIN 4
    #define NUM_LEDS 1
    CRGB leds[NUM_LEDS];
#endif
//========================================//


// GSM settings
#define SerialAT Serial1
#define MODEM_TX 17
#define MODEM_RX 16
#define MODEM_PWR 15
#define SIM_BAUD 115200

const char apn[] = "internet"; // APN
const char apnUser[] = "";
const char apnPass[] = "";
const char* broker = "broker2.dma-bd.com";
const char* mqttUser = "broker2";
const char* mqttPass = "Secret!@#$1234";
bool gsmConnected = false;
//========================================//

// OTA server (default) - used when OTA command doesn't supply a URL
const char* otaHostDefault = "iot2.dma-bd.com";
const int otaPortDefault = 5000;
const char* otaPathDefault = "/download/MC251015.bin";

#define NETWORK_TASK_PRIORITY 3
#define OTA_TASK_STACK_SIZE     (16 * 1024)
//========================================//


// MQTT settings
char mqttSubTopic[64]; 
#define MQTT_PORT 1883
#define MQTT_MC_PUB "DMA/A7670/PUB"
#define MQTT_MC_SUB "DMA/A7670/SUB"
#define MQTT_MC_HB "DMA/A7670/HB"
#define MQTT_OTA_PUB "DMA/A7670/OTA"

//Struct to hold message data
#define MAX_MQTT_MSG_LEN 128
#define MAX_TOPIC_LEN    64

typedef struct {
    char topic[MAX_TOPIC_LEN];
    char payload[MAX_MQTT_MSG_LEN];
} MqttMessage;


typedef struct {
    CRGB color;
    uint16_t duration;  // ms
    uint8_t repeat;     // number of times to blink
    uint16_t gap;       // optional gap between blinks
} LedBlink;
//========================================//
