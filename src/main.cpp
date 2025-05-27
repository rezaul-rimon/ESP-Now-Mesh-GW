#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const char* gatewayID = "GW01";  // ✅ Unique Gateway ID

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

  Serial.println("Gateway Ready.");
}

void loop() {
  String msg = String(gatewayID) + "," + "Hello ESP-NOW";  // 👈 Add gateway ID
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)msg.c_str(), msg.length());
  Serial.println(result == ESP_OK ? "✅ Sent!" : "❌ Send Failed");
  delay(3000);
}
