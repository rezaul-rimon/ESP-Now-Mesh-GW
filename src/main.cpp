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

// #include <semphr.h>   // For FreeRTOS semaphore (optional on some platforms)


unsigned long lastDataPublishTime = 0;
const unsigned long dataPublishInterval = 60000; //

unsigned long lastHBPublishTime = 0;
const unsigned long hbPublishInterval = 30000; // 60 seconds

//Function prototypes
void networkTask(void *param); 
void mainTask(void *param);
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool connectGSM();
void reconnectMqtt();
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
String generateMessageID();

//Gateway configuration
const char* DEVICE_ID = "1191032505290001";
const char* Local_ID = "GW0"; // Gateway ID
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


//FreeRTOS Task instance
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
#define MQTT_HB "DMA/EM/HB"
#define MQTT_PUB "DMA/EM/PUB"
#define MQTT_SUB "DMA/EM/SUB"

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
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "sim7600_%04X", random(0xffff));
    Serial.print("[MQTT] Connecting as client ID: ");
    Serial.println(clientId);

    if (mqtt.connect(clientId, mqttUser, mqttPass)) {
      Serial.println("[MQTT] Connected");
      snprintf(mqttSubTopic, sizeof(mqttSubTopic), "%s/%s", MQTT_SUB, DEVICE_ID);
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

// Callback function for ESP-NOW messages
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

    if (type != "ack" || gw_id != Local_ID) return;
  } else if (commaCount == 3) {
    int idx1 = msg.indexOf(',');
    int idx2 = msg.indexOf(',', idx1 + 1);
    int idx3 = msg.indexOf(',', idx2 + 1);

    node_id = msg.substring(0, idx1);
    command = msg.substring(idx1 + 1, idx2);
    type    = msg.substring(idx2 + 1, idx3);
    msg_id  = msg.substring(idx3 + 1);

    if (type != "ack") return;
  } else return;

  if (isDuplicateACK(msg_id)) {
    Serial.println("⚠️ Duplicate ACK ignored");
    return;
  }

  Serial.printf("✅ ACK Received: node=%s cmd=%s id=%s\n", node_id.c_str(), command.c_str(), msg_id.c_str());
}

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
  delay(2000);                    // Wait 2 seconds
  digitalWrite(MODEM_PWR, HIGH);  // Turn on GSM
  delay(3000);                    // Wait 3 seconds for boot
}

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
      Serial.println("⚠️ Could not acquire modem mutex");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Main task to handle user input and send commands
void mainTask(void *param) {
  for (;;) {
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


    // Heartbeat publish
    if (millis() - lastHBPublishTime >= hbPublishInterval) {
      lastHBPublishTime = millis();

      if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000))) {
        
        if(mqtt.publish(MQTT_HB, "💓 Heartbeat from Gateway")) {
          Serial.println("[MQTT] Heartbeat sent");
        } else {
          Serial.println("[MQTT] Failed to send heartbeat");
        }

        xSemaphoreGive(modemMutex);
      } else {
        Serial.println("⚠️ Could not acquire modem mutex to send heartbeat");
      }
    }

    // Modbus Data publish
    if (millis() - lastDataPublishTime >= dataPublishInterval) {
      lastDataPublishTime = millis();

      getModbusData(); // Read Modbus data
      ParsingModbusData(); // Prepare data string
      Serial.println("📤 Publishing Modbus Data....");
      if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000))) {

        if(mqtt.publish(MQTT_PUB, em_data)) {
          Serial.println("[MQTT] Data sent: " + String(em_data));
        } else {
          Serial.println("[MQTT] Failed to send data");
        }

        xSemaphoreGive(modemMutex);
      } else {
        Serial.println("⚠️ Could not acquire modem mutex to send heartbeat");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Prevent WDT
  }
}


void setup() {
  Serial.begin(115200);
  SerialAT.begin(SIM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(MODEM_PWR, OUTPUT);
  digitalWrite(MODEM_PWR, HIGH);

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

  modemMutex = xSemaphoreCreateMutex();
  if (modemMutex == NULL) {
    Serial.println("❌ Failed to create modemMutex");
    while (true); // halt
  }

  Serial.println("✅ Gateway Ready. Enter node_id,command to send:");

  xTaskCreatePinnedToCore(networkTask, "Network Task", 8 * 1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16 * 1024, NULL, 1, &mainTaskHandle, 1);
}

void loop() {
  
}
