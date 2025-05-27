#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const char* gatewayID = "GW01";

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg = "";
  for (int i = 0; i < len; i++) {
    msg += (char)incomingData[i];
  }

  // 📥 Print message from node
  int separator = msg.indexOf(',');
  if (separator != -1) {
    String nodeID = msg.substring(0, separator);
    String content = msg.substring(separator + 1);
    Serial.print("📡 From Node: ");
    Serial.println(nodeID);
    Serial.print("📩 Message: ");
    Serial.println(content);
  } else {
    Serial.println("⚠️ Malformed message from node.");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("Broadcast peer added.");
  }

  esp_now_register_recv_cb(onReceive);  // ✅ Enable receiving from nodes

  Serial.println("Gateway Ready.");
}

void loop() {
  String msg = String(gatewayID) + "," + "hello-node";
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)msg.c_str(), msg.length());
  Serial.println(result == ESP_OK ? "✅ Sent hello-node!" : "❌ Send Failed");
  delay(5000);  // ⏳ Every 5 seconds
}
