
// This code is designed to run on an ESP32 device with a SIM7600 GSM module.
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define USE_SD_CARD false

//Libraries required for GSM, MQTT, and ESP-NOW functionality
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <esp_now.h>
#include <deque>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <FastLED.h>

// #define DEBUG_MODE true
// #define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
// #define DEBUG_PRINTF(x)  if (DEBUG_MODE) { Serial.printf(x); }
// #define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }

//Gateway configuration
const char* DEVICE_ID = "1191032506169999";

const char* Local_ID = "gw0"; // Gateway ID
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Timers for publishing data and heartbeat
unsigned long lastDataPublishTime = 0;
const unsigned long dataPublishInterval = 5 * 60 * 1000;

unsigned long lastHBPublishTime = 0;
const unsigned long hbPublishInterval = 2 * 60 * 1000;

unsigned long lastHourCheck = 0;
bool snapshotSentThisHour = false;

//FastLED library for controlling LEDs
#define LED_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
#define AC_LINE_PIN 34

// GSM settings
#define SerialAT Serial1
#define MODEM_TX 17
#define MODEM_RX 16
#define MODEM_PWR 15
#define SIM_BAUD 115200

const char apn[] = "blweb";
const char user[] = "";
const char pass[] = "";
const char* broker = "broker2.dma-bd.com";
const char* mqttUser = "broker2";
const char* mqttPass = "Secret!@#$1234";
bool gsmConnected = false;

// MQTT settings
char mqttSubTopic[64]; 
#define MQTT_PORT 1883
#define MQTT_EM_HB "DMA/EM/HB"
#define MQTT_EM_PUB "DMA/EM/PUB"
#define MQTT_AC_HB "DMA/MeshAC/HB"
#define MQTT_AC_SUB "DMA/MeshAC/SUB"
#define MQTT_AC_ACK "DMA/MeshAC/ACK"
#define MQTT_AC_TMP "DMA/MeshAC/TEMP"
// #define MQTT_CMD "DMA/AC/CMD"

// RS485 Serial2 Pins
#define RS485_RX 27
#define RS485_TX 14

// Modbus register addresses
#define taeHigh_reg_addr     0x30
#define taeLow_reg_addr      0x31
#define activePower_reg_addr 0x1A
#define pAvolt_reg_addr      0x14
#define pBvolt_reg_addr      0x15
#define pCvolt_reg_addr      0x16
#define lABvolt_reg_addr     0x17
#define lBCvolt_reg_addr     0x18
#define lCAvolt_reg_addr     0x19
#define pAcurrent_reg_addr   0x10
#define pBcurrent_reg_addr   0x11
#define pCcurrent_reg_addr   0x12
#define frequency_reg_addr   0x1E
#define powerfactor_reg_addr 0x1D

// Data variables
int taeHigh, taeLow, activePower;
int pAvolt, pBvolt, pCvolt;
int lABvolt, lBCvolt, lCAvolt;
int pAcurrent, pBcurrent, pCcurrent;
int frequency, powerFactor;

char em_data[128];
ModbusMaster node;

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
