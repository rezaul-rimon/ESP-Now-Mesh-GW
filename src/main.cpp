// ✅ GATEWAY CODE (ESP-NOW CMD SENDER + ACK RECEIVER)
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

#include <WiFi.h>
#include <esp_now.h>
#include <deque>
#include <algorithm>


const char* DEVICE_ID = "GW0";
char mqttSubTopic[64]; 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// GSM settings
#define SerialAT Serial1
#define MODEM_TX 17
#define MODEM_RX 16
#define MODEM_PWR 15
#define SIM_BAUD 115200
#define MQTT_PORT 1883
#define MQTT_HB "DMA/EM/HB"
#define MQTT_PUB "DMA/EM/PUB"
#define MQTT_SUB "DMA/EM/SUB"

const char apn[] = "blweb";
const char user[] = "";
const char pass[] = "";
const char* broker = "broker2.dma-bd.com";
const char* mqttUser = "broker2";
const char* mqttPass = "Secret!@#$1234";

TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

struct Message {
  String gw_id;
  String node_id;
  String command;
  String type;
  String msg_id;
};

std::deque<String> recentAckIDs;
const size_t maxRecentIDs = 20;

bool connectGSM() {
  Serial.println("[GSM] Initializing modem...");
  modem.restart();
  delay(3000);
  if (modem.getSimStatus() != SIM_READY) return false;
  Serial.println("[GSM] Connecting to network...");
  if (!modem.waitForNetwork()) return false;
  if (!modem.gprsConnect(apn)) return false;
  Serial.println("[GSM] Connected to GPRS!");
  return true;
}

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

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("[MQTT IN] Topic: " + String(topic));
  Serial.println("[MQTT IN] Message: " + message);

  // You can add command handling here if needed
}

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

String generateMessageID() {
  uint32_t randNum = esp_random();
  char id[9];
  sprintf(id, "%08X", randNum);
  return String(id);
}

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

    if (type != "ack" || gw_id != DEVICE_ID) return;
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
}

void loop() {
  mqtt.loop();

  if (!modem.isGprsConnected()) connectGSM();
  if (!mqtt.connected()) reconnectMqtt();

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
    msg.gw_id = DEVICE_ID;
    msg.node_id = input.substring(0, commaIndex);
    msg.command = input.substring(commaIndex + 1);
    msg.type = "cmd";
    msg.msg_id = generateMessageID();

    String payload = msg.gw_id + "," + msg.node_id + "," + msg.command + "," + msg.type + "," + msg.msg_id;
    esp_now_send(broadcastAddress, (uint8_t*)payload.c_str(), payload.length());
    Serial.println("📤 CMD Sent: " + payload);
  }
}
