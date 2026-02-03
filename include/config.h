
// This code is designed to run on an ESP32 device with a SIM7600 GSM module.
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
//======================================//

// === Project Configuration === //
// #define USE_SD_CARD
#define USE_FastLED
// #define USE_LDR_SENSOR
// #define USE_GY30
// #define USE_RCSWITCH
//======================================//

// Include RCSwitch library for RF communication
#ifdef USE_RCSWITCH
    #include <RCSwitch.h>
    RCSwitch mySwitch = RCSwitch();

    #include <map>
    std::map<unsigned long, unsigned long> lastRFReceivedTimeMap;
    unsigned long lastRFGlobalReceivedTime = 0;  // Global debounce

    #define RF_PIN 26

    unsigned int RF_Remote_1 = 0; // Example RF code for Remote 1
    unsigned int RF_Remote_2 = 0; // Example RF code for Remote 2
    unsigned int RF_Remote_3 = 0; // Example RF code for Remote 3
    unsigned int RF_Remote_4 = 0; // Example RF code for Remote 4
    unsigned int RF_Remote_5 = 0; // Example RF code for Remote 5
#endif
//======================================

//Libraries required for GSM, MQTT, and ESP-NOW functionality
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <deque>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <esp_task_wdt.h>
#include <FastLED.h>
#include <Preferences.h>
#include <Update.h>
#include <Wire.h>

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
    #define WORK_PACKAGE "1225"
    #define GW_TYPE "03"
    #define FIRMWARE_UPDATE_DATE "260121" 
    #define DEVICE_SERIAL "0010"
#endif

const char* DEVICE_ID;
//========================================//

const char* Local_ID = "gw1"; // Gateway ID
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Timers for publishing data and heartbeat
unsigned long lastDataPublishTime = 0;
const unsigned long dataPublishInterval = 5 * 60 * 1000;

unsigned long lastHBPublishTime = 0;
const unsigned long hbPublishInterval = 5 * 60 * 1000;

unsigned long lastHourCheck = 0;
bool snapshotSentThisHour = false;

bool ledState = false;
//========================================//

//FastLED library for controlling LEDs
#ifdef USE_FastLED
    #define LED_PIN 4
    #define NUM_LEDS 1
    CRGB leds[NUM_LEDS];
#endif
//========================================//

// Sensor configuration
#ifdef USE_LDR_SENSOR
    #define LDR_PIN 39 // Pin for LDR sensor
#endif

#ifdef USE_GY30
    #include <BH1750.h>
    BH1750 lightMeter(0x23);
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
const char* otaPathDefault = "/download/MeshGWLP260121.bin";

#define NETWORK_TASK_PRIORITY 3
#define OTA_TASK_STACK_SIZE     (16 * 1024)
//========================================//


// MQTT settings
char mqttSubTopic[64]; 
#define MQTT_PORT 1883
#define MQTT_MC_PUB "DMA/MeshGWLP/PUB"
#define MQTT_MC_SUB "DMA/MeshGWLP/SUB"
#define MQTT_MC_HB "DMA/MeshGWLP/HB"
#define MQTT_OTA_PUB "DMA/MeshGWLP/OTA"

#define MQTT_SMARTSWITCH_HB "DMA/SmartSwitchLP/HB"
#define MQTT_SMARTSWITCH_ACK "DMA/SmartSwitchLP/ACK"

//Struct to hold message data
#define MAX_MQTT_MSG_LEN 128
#define MAX_TOPIC_LEN    64

typedef struct {
    char topic[MAX_TOPIC_LEN];
    char payload[MAX_MQTT_MSG_LEN];
} MqttMessage;

struct Message {
    String sender_id;
    String receiver_id;
    String command;
    String type;
    String msg_id;
};

typedef struct {
    CRGB color;
    uint16_t duration;  // ms
    uint8_t repeat;     // number of times to blink
    uint16_t gap;       // optional gap between blinks
} LedBlink;


std::deque<String> recentMsgKeys;
const size_t maxRecentIDs = 20;
