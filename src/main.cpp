// ✅ GATEWAY CODE (ESP-NOW CMD SENDER + ACK RECEIVER)

// This code is designed to run on an ESP32 device with a SIM7600 GSM module.
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

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

//FastLED library for controlling LEDs
#define LED_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
#define AC_LINE_PIN 34

//Timers for publishing data and heartbeat
unsigned long lastDataPublishTime = 0;
const unsigned long dataPublishInterval = 5 * 60 * 1000;

unsigned long lastHBPublishTime = 0;
const unsigned long hbPublishInterval = 2 * 60 * 1000;

unsigned long lastHourCheck = 0;
bool snapshotSentThisHour = false;

//Function prototypes
void networkTask(void *param); 
void mainTask(void *param);
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool connectGSM();
void reconnectMqtt();
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
String generateMessageID();

//Gateway configuration
const char* DEVICE_ID = "1191032506010001";
const char* Local_ID = "gw1"; // Gateway ID
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


//FreeRTOS Tasks and instance
QueueHandle_t mqttQueue;
SemaphoreHandle_t modemMutex;
TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
// TaskHandle_t wifiResetTaskHandle;

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
#define MQTT_EM_HB "DMA/EnergyMeter/HB"
#define MQTT_EM_PUB "DMA/EnergyMeter/PUB"
#define MQTT_AC_HB "DMA/AC/HB"
#define MQTT_AC_PUB "DMA/AC/PUB"
#define MQTT_AC_SUB "DMA/AC/SUB"
#define MQTT_AC_ACK "DMA/AC/ACK"
// #define MQTT_CMD "DMA/AC/CMD"

//Objects for GSM, MQTT, and ESP-NOW
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

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
  String gw_id;
  String node_id;
  String command;
  String type;
  String msg_id;
};

//Queue to hold recent ACK IDs
std::deque<String> recentAckIDs;
const size_t maxRecentIDs = 20;

// Function to connect to GSM network
bool connectGSM() {
  Serial.println("[GSM] Initializing modem...");
  modem.restart();
  vTaskDelay(pdMS_TO_TICKS(3000)); // Adjust delay as needed
  
  if (modem.getSimStatus() != SIM_READY) return false;
  Serial.println("[GSM] Connecting to network...");
  if (!modem.waitForNetwork()) return false;
  if (!modem.gprsConnect(apn)) return false;
  Serial.println("[GSM] Connected to GPRS!");
  return true;
}

// Function to reconnect to MQTT broker
void reconnectMqtt() {
  if (!mqtt.connected() && gsmConnected) {
    leds[0]=CRGB::Orange; 
    FastLED.show();
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "ac_gsm_%04X%04X", random(0xffff), random(0xffff));
    Serial.print("[MQTT] Connecting as client ID: ");
    Serial.println(clientId);

    if (mqtt.connect(clientId, mqttUser, mqttPass)) {
      Serial.println("[MQTT] Connected");
      leds[0]=CRGB::Black; 
      FastLED.show();
      snprintf(mqttSubTopic, sizeof(mqttSubTopic), "%s/%s", MQTT_AC_SUB, DEVICE_ID);
      mqtt.subscribe(mqttSubTopic);
      Serial.print("[MQTT] Subscribed to topic: ");
      Serial.println(mqttSubTopic);
    } else {
      Serial.print("[MQTT] Failed, rc=");
      Serial.println(mqtt.state());
    }
  }
}

// Callback function for MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  leds[0]=CRGB::Blue; 
  FastLED.show();
  vTaskDelay(pdMS_TO_TICKS(100));
  leds[0]=CRGB::Black;
  FastLED.show();

  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("[MQTT IN] Topic: " + String(topic));
  Serial.println("[MQTT IN] Message: " + message);

  message.trim();           // Removes leading/trailing whitespace
  message.replace(" ", ""); // Removes all internal spaces
  Serial.println("📥 Message: " + message);

  int commaIndex = message.indexOf(',');
  if (commaIndex < 0) {
    Serial.println("⚠️ Format: node_id,command");
    leds[0]=CRGB::Orange; 
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(100));
    leds[0]=CRGB::Black;
    FastLED.show();
    return;
  }

  Message msg;
  msg.gw_id = Local_ID;
  msg.node_id = message.substring(0, commaIndex);
  msg.command = message.substring(commaIndex + 1);
  msg.type = "cmd";
  msg.msg_id = generateMessageID();

  String payload2 = msg.gw_id + "," + msg.node_id + "," + msg.command + "," + msg.type + "," + msg.msg_id;
  esp_now_send(broadcastAddress, (uint8_t*)payload2.c_str(), payload2.length());
  Serial.println("📤 CMD Sent: " + payload2);

  // You can add command handling here if needed
}

// Function to read Modbus data from the RS485 slave
int readModbusData(uint16_t reg_address, uint8_t max_retries) {
  vTaskDelay(pdMS_TO_TICKS(30));
  int value = -1;
  while (max_retries-- > 0) {
    uint8_t result = node.readHoldingRegisters(reg_address, 1);
    if (result == node.ku8MBSuccess) {
      value = node.getResponseBuffer(0);
      Serial.print("Modbus Read 0x");
      Serial.print(reg_address, HEX);
      Serial.print(": ");
      Serial.println(value);
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(60));
  }
  return value;
}

// Function to initialize Modbus communication
void ParsingModbusData() {
  snprintf(em_data, sizeof(em_data),
    "%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    DEVICE_ID,
    taeHigh, taeLow, activePower,
    pAvolt, pBvolt, pCvolt,
    lABvolt, lBCvolt, lCAvolt,
    pAcurrent, pBcurrent, pCcurrent,
    frequency, powerFactor);
}

// Function to check for duplicate ACKs
bool isDuplicateACK(const String& msg_id) {
  if (std::find(recentAckIDs.begin(), recentAckIDs.end(), msg_id) != recentAckIDs.end()) {
    return true;
  }
  recentAckIDs.push_back(msg_id);
  if (recentAckIDs.size() > maxRecentIDs) {
    recentAckIDs.pop_front();
  }
  return false;
}

// Function to generate a unique message ID
String generateMessageID() {
  uint32_t randNum = esp_random();
  char id[5];
  sprintf(id, "%04X", randNum);
  return String(id);
}

//Callback function for ESP-NOW messages
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg((char*)incomingData, len);
  Serial.println("\n📥 Received: " + msg);

  int commaCount = std::count(msg.begin(), msg.end(), ',');

  String node_id, command, type, msg_id, gw_id;
  if (commaCount == 4) {
    int idx1 = msg.indexOf(',');
    int idx2 = msg.indexOf(',', idx1 + 1);
    int idx3 = msg.indexOf(',', idx2 + 1);
    int idx4 = msg.indexOf(',', idx3 + 1);

    gw_id   = msg.substring(0, idx1);
    node_id = msg.substring(idx1 + 1, idx2);
    command = msg.substring(idx2 + 1, idx3);
    type    = msg.substring(idx3 + 1, idx4);
    msg_id  = msg.substring(idx4 + 1);

    if (type != "ack" && type != "hb") return;

    } else if (commaCount == 3) {
      int idx1 = msg.indexOf(',');
      int idx2 = msg.indexOf(',', idx1 + 1);
      int idx3 = msg.indexOf(',', idx2 + 1);

      node_id = msg.substring(0, idx1);
      command = msg.substring(idx1 + 1, idx2);
      type    = msg.substring(idx2 + 1, idx3);
      msg_id  = msg.substring(idx3 + 1);

      if (type != "ack" && type != "hb") return;

    } else return;

    // ✅ Deduplication
    if (isDuplicateACK(msg_id)) {
      Serial.println("⚠️ Duplicate " + type + " ignored");
      return;
    }

    // ✅ Print parsed data
    Serial.printf("✅ %s Received: node=%s cmd=%s id=%s\n", type.c_str(), node_id.c_str(), command.c_str(), msg_id.c_str());

    // ✅ Push to MQTT queue
    MqttMessage mqttMsg;

    if (type == "ack") {
      snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_AC_ACK);  // your predefined topic
      snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, node_id.c_str(), command.c_str());

    } else if (type == "hb") {
      snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_AC_HB); // you define this topic
      snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, node_id.c_str(), command.c_str());
    }

    xQueueSend(mqttQueue, &mqttMsg, 0);
}

// Function to initialize Modbus communication
void getModbusData(){
  taeHigh     = readModbusData(taeHigh_reg_addr, 3);
  taeLow      = readModbusData(taeLow_reg_addr, 3);
  activePower = readModbusData(activePower_reg_addr, 2);
  pAvolt      = readModbusData(pAvolt_reg_addr, 1);
  pBvolt      = readModbusData(pBvolt_reg_addr, 1);
  pCvolt      = readModbusData(pCvolt_reg_addr, 2);
  lABvolt     = readModbusData(lABvolt_reg_addr, 1);
  lBCvolt     = readModbusData(lBCvolt_reg_addr, 1);
  lCAvolt     = readModbusData(lCAvolt_reg_addr, 1);
  pAcurrent   = readModbusData(pAcurrent_reg_addr, 1);
  pBcurrent   = readModbusData(pBcurrent_reg_addr, 1);
  pCcurrent   = readModbusData(pCcurrent_reg_addr, 1);
  frequency   = readModbusData(frequency_reg_addr, 1);
  powerFactor = readModbusData(powerfactor_reg_addr, 1);

  Serial.println("--------- Modbus Data ---------");
  Serial.printf("taeHigh: %d\n", taeHigh);
  Serial.printf("taeLow: %d\n", taeLow);
  Serial.printf("Active Power: %d\n", activePower);
  Serial.printf("Phase A Voltage: %d\n", pAvolt);
  Serial.printf("Phase B Voltage: %d\n", pBvolt);
  Serial.printf("Phase C Voltage: %d\n", pCvolt);
  Serial.printf("Line AB Voltage: %d\n", lABvolt);
  Serial.printf("Line BC Voltage: %d\n", lBCvolt);
  Serial.printf("Line CA Voltage: %d\n", lCAvolt);
  Serial.printf("Phase A Current: %d\n", pAcurrent);
  Serial.printf("Phase B Current: %d\n", pBcurrent);
  Serial.printf("Phase C Current: %d\n", pCcurrent);
  Serial.printf("Frequency: %d Hz\n", frequency);
  Serial.printf("Power Factor: %d\n", powerFactor);
  Serial.println("--------------------------------");
}

// Function to power cycle the GSM module
void powerCycleGSM() {
  Serial.println("🔁 Power cycling GSM module...");
  digitalWrite(MODEM_PWR, LOW);   // Turn off GSM
  leds[0]=CRGB::Red; 
  FastLED.show();
  delay(2000);                    // Wait 2 seconds
  digitalWrite(MODEM_PWR, HIGH);  // Turn on GSM
  leds[0]=CRGB::Orange; 
  FastLED.show();
  delay(3000);
  leds[0]=CRGB::Black; 
  FastLED.show();                    // Wait 3 seconds for boot
}

// Setup function to initialize everything
void networkTask(void *param) {
  enum GsmState { GSM_INIT, GSM_CONNECTING, GSM_CONNECTED, GSM_ERROR };
  GsmState gsmState = GSM_INIT;
  // bool gsmConnected = false;

  for (;;) {
    if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(200))) {
      mqtt.loop();

      switch (gsmState) {
        case GSM_INIT:
          Serial.println("[GSM] Restarting modem...");
          leds[0]=CRGB::Red; 
          FastLED.show();
          modem.restart();
          delay(2000);
          gsmState = GSM_CONNECTING;
          break;

        case GSM_CONNECTING:
          Serial.println("[GSM] Checking SIM and Network...");
          if (modem.getSimStatus() == SIM_READY && modem.waitForNetwork()) {
            if (modem.gprsConnect(apn)) {
              Serial.println("[GSM] Connected to GPRS");
              gsmConnected = true;
              gsmState = GSM_CONNECTED;
            } else {
              Serial.println("❌ Failed to connect GPRS");
              gsmState = GSM_ERROR;
            }
          } else {
            Serial.println("❌ SIM or Network not ready");
            gsmState = GSM_ERROR;
          }
          break;

        case GSM_CONNECTED:
          if (!modem.isGprsConnected()) {
            Serial.println("❌ Lost GPRS connection");
            gsmConnected = false;
            gsmState = GSM_ERROR;
          } else if (!mqtt.connected()) {
            reconnectMqtt();
          }
          break;

        case GSM_ERROR:
          Serial.println("[GSM] GSM Error. Power cycling...");
          gsmConnected = false;

          // 🔁 Try hardware reset
          powerCycleGSM();

          vTaskDelay(pdMS_TO_TICKS(2000));
          gsmState = GSM_INIT;
          break;
      }

      xSemaphoreGive(modemMutex);
    } else {
      Serial.println("⚠️ Could not acquire modem mutex, from networkTask");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Main task to handle serial commands, heartbeat, and Modbus data
void mainTask(void *param) {
  for (;;) {
    // 📥 Serial command handler
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim(); input.replace(" ", "");
      Serial.println("📥 Input: " + input);

      int commaIndex = input.indexOf(',');
      if (commaIndex < 0) {
        Serial.println("⚠️ Format: node_id,command");
      } else {
        Message msg;
        msg.gw_id = Local_ID;
        msg.node_id = input.substring(0, commaIndex);
        msg.command = input.substring(commaIndex + 1);
        msg.type = "cmd";
        msg.msg_id = generateMessageID();

        String payload = msg.gw_id + "," + msg.node_id + "," + msg.command + "," + msg.type + "," + msg.msg_id;
        esp_now_send(broadcastAddress, (uint8_t*)payload.c_str(), payload.length());
        Serial.println("📤 CMD Sent: " + payload);
      }
    }

    // 💓 Heartbeat via MQTT queue
    if (millis() - lastHBPublishTime >= hbPublishInterval) {
    lastHBPublishTime = millis();
    bool acLineState = digitalRead(AC_LINE_PIN);

    MqttMessage hbMsg;
    snprintf(hbMsg.topic, MAX_TOPIC_LEN, MQTT_EM_PUB);
    snprintf(hbMsg.payload, MAX_MQTT_MSG_LEN, "%s,W:1,G:0,C:%d,SD:0", DEVICE_ID, acLineState ? 1 : 0);

    xQueueSend(mqttQueue, &hbMsg, 0);
  }

    // 📊 Modbus Data via MQTT queue
    if (millis() - lastDataPublishTime >= dataPublishInterval) {
      lastDataPublishTime = millis();

      getModbusData();         // Populate raw data
      ParsingModbusData();     // Format to em_data

      Serial.println("📤 Publishing Modbus Data....");

      MqttMessage dataMsg;
      snprintf(dataMsg.topic, MAX_TOPIC_LEN, MQTT_EM_PUB);
      snprintf(dataMsg.payload, MAX_MQTT_MSG_LEN, "%s", em_data);

      xQueueSend(mqttQueue, &dataMsg, 0);
    }

    // 📅 Hourly snapshot
    // unsigned long now = millis();
    // // Hourly snapshot logic
    // if (now - lastHourCheck >= 60 * 1000) {  // Check every 1 min
    //   lastHourCheck = now;

    //   if (isTopOfHour()) {
    //     if (!snapshotSentThisHour) {
    //       snapshotSentThisHour = true;

    //       getModbusData();         // Populate raw data
    //       ParsingModbusData();     // Format to em_data

    //       // Send hourly snapshot
    //       String snapshotData = "📸 Hourly Snapshot at top of hour";
    //       if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000))) {
    //         mqtt.publish("iot/hourly", snapshotData.c_str());
    //         Serial.println("[MQTT] 📸 Hourly snapshot sent");
    //         xSemaphoreGive(modemMutex);
    //       } else {
    //         Serial.println("⚠️ Could not acquire modem mutex for hourly snapshot");
    //       }
    //     }
    //   } else {
    //     snapshotSentThisHour = false;  // reset flag once out of top of hour
    //   }
    // }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Yield for watchdog
  }
}

void mqttPublishTask(void *param) {
  MqttMessage msg;

  for (;;) {
    if (xQueueReceive(mqttQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000))) {
        if (mqtt.publish(msg.topic, msg.payload)) {
          Serial.printf("[MQTT] Published to %s: %s\n", msg.topic, msg.payload);

          // 🔘 LED indication based on topic
          if (String(msg.topic) == MQTT_EM_HB) {
            leds[0] = CRGB::Blue;
            FastLED.show();
            vTaskDelay(pdMS_TO_TICKS(500));
          } else if(msg.topic == MQTT_EM_PUB) {
            leds[0] = CRGB::Green;
            FastLED.show();
            vTaskDelay(pdMS_TO_TICKS(1000));
          }
          leds[0] = CRGB::Black;
          FastLED.show();

        } else {
          Serial.printf("[MQTT] Failed to publish to %s\n", msg.topic);
        }
        xSemaphoreGive(modemMutex);
      } else {
        Serial.println("⚠️ Could not acquire modem mutex in mqttPublishTask");
      }
    }
  }
}

// Function to check if it's the top of the hour
void setup() {
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL,LED_PIN>(leds,NUM_LEDS);
  
  leds[0]=CRGB::Red; 
  FastLED.show();
  delay(250);
  leds[0]=CRGB::Yellow;
  FastLED.show();
  delay(250);
  leds[0]=CRGB::Blue;
  FastLED.show();
  delay(250);
  leds[0]=CRGB::Black;
  FastLED.show();
  Serial.println("🔄 Starting Gateway...");


  SerialAT.begin(SIM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(MODEM_PWR, OUTPUT);
  digitalWrite(MODEM_PWR, HIGH);

  pinMode(AC_LINE_PIN, INPUT); // AC line detection pin

  mqtt.setServer(broker, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(onReceive);

  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  node.begin(1, Serial2);

  mqttQueue = xQueueCreate(10, sizeof(MqttMessage));
  if (mqttQueue == NULL) {
    Serial.println("❌ Failed to create mqttQueue");
    while (1); // Stop here if failed
  }

  modemMutex = xSemaphoreCreateMutex();
  if (modemMutex == NULL) {
    Serial.println("❌ Failed to create modemMutex");
    while (true); // halt
  }

  Serial.println("✅ Gateway Ready. Enter node_id,command to send:");

  xTaskCreatePinnedToCore(networkTask, "Network Task", 8 * 1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16 * 1024, NULL, 1, &mainTaskHandle, 1);
  xTaskCreatePinnedToCore(mqttPublishTask, "MQTT Pub Task", 6 * 1024, NULL, 1, NULL, 1);

}

// Function to check if it's the top of the hour
void loop() {
  
}
