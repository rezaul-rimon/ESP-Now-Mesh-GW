#include <WiFi.h>
#include <esp_now.h>

const char* gatewayID = "GW0";
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

String inputBuffer = "";

// 📥 Callback when data is received
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg = "";
  for (int i = 0; i < len; i++) {
    msg += (char)incomingData[i];
  }

  // Expected format: NODE01,some reply text
  int firstComma = msg.indexOf(',');
  if (firstComma > 0) {
    String nodeID = msg.substring(0, firstComma);
    String content = msg.substring(firstComma + 1);
    Serial.print("📥 Response from ");
    Serial.print(nodeID);
    Serial.print(": ");
    Serial.println(content);
  } else {
    Serial.print("📥 Unknown message: ");
    Serial.println(msg);
  }
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
  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("✅ Broadcast peer added.");
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("✅ Gateway Ready. Type: NODE_ID,message");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        String finalMessage = String(gatewayID) + "," + inputBuffer;
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)finalMessage.c_str(), finalMessage.length());
        Serial.print("📤 Sending: ");
        Serial.println(finalMessage);
        Serial.println(result == ESP_OK ? "✅ Sent!" : "❌ Send Failed");
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}
