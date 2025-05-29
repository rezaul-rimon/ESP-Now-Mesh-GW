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
#include <WiFi.h>
#include <esp_now.h>
#include <deque>
#include <algorithm>
#include <freertos/FreeRTOS.h>

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
  if (!mqtt.connected()) {
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "sim7600_%04X", random(0xffff));
    Serial.print("[MQTT] Connecting as client ID: ");
    Serial.println(clientId);
    if (mqtt.connect(clientId, mqttUser, mqttPass)) {
      Serial.println("[MQTT] Connected");

      // Prepare and subscribe to topic
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


// Task to handle network operations (GSM and MQTT)
void networkTask(void *param) {
  for (;;) {
    mqtt.loop();
    if (!modem.isGprsConnected()) connectGSM();
    if (!mqtt.connected()) reconnectMqtt();
    vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
  }
}

// Main task to handle user input and send commands
void mainTask(void *param) {
  for (;;) {

    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();           // Removes leading/trailing whitespace
      input.replace(" ", ""); // Removes all internal spaces
      Serial.println("📥 Input: " + input);

      int commaIndex = input.indexOf(',');
      if (commaIndex < 0) {
        Serial.println("⚠️ Format: node_id,command");
        return;
      }

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

    vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
  }
}

void setup() {
  Serial.begin(115200);

  SerialAT.begin(SIM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(MODEM_PWR, OUTPUT);
  digitalWrite(MODEM_PWR, HIGH);

  for (int i = 0; i < 5 && !connectGSM(); i++) {
    Serial.println("[GSM] Retry connecting...");
    delay(5000);
  }

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

  Serial.println("✅ Gateway Ready. Enter node_id,command to send:");

  xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, &mainTaskHandle, 1);
  // xTaskCreatePinnedToCore(wifiResetTask, "WiFi Reset Task", 4*1024, NULL, 1, &wifiResetTaskHandle, 1);
}

void loop() {
  
}
