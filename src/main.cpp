// ✅ GATEWAY CODE (ESP-NOW CMD SENDER + ACK RECEIVER)
#include <WiFi.h>
#include <esp_now.h>
#include <deque>
#include <algorithm>

const char* gatewayID = "GW0";
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct Message {
  String gw_id;
  String node_id;
  String command;
  String type;
  String msg_id;
};

std::deque<String> recentAckIDs;
const size_t maxRecentIDs = 20;

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

    if (type != "ack" || gw_id != gatewayID) return;
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
    msg.gw_id = gatewayID;
    msg.node_id = input.substring(0, commaIndex);
    msg.command = input.substring(commaIndex + 1);
    msg.type = "cmd";
    msg.msg_id = generateMessageID();

    String payload = msg.gw_id + "," + msg.node_id + "," + msg.command + "," + msg.type + "," + msg.msg_id;
    esp_now_send(broadcastAddress, (uint8_t*)payload.c_str(), payload.length());
    Serial.println("📤 CMD Sent: " + payload);
  }
}
