#include <WiFi.h>
#include <esp_now.h>

// ------------------------------
// 📌 Constants & Configuration
// ------------------------------
const char* gatewayID = "GW0";
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ------------------------------
// 📦 Message Structure
// ------------------------------
struct Message {
  String gw_id;     // Gateway ID
  String node_id;   // Destination Node ID
  String command;   // Command string (e.g., red, blue)
  String type;      // "cmd" for command, "ack" for acknowledgment
  String msg_id;    // Unique 8-digit message ID
};

// ------------------------------
// 🎲 Generate 8-digit Hex Message ID
// ------------------------------
String generateMessageID() {
  uint32_t randNum = esp_random();  // Or use random(0xFFFFFFFF)
  char id[9];
  sprintf(id, "%08X", randNum);     // Uppercase hex, 8 chars
  return String(id);
}

// ------------------------------
// 📥 Callback on Receiving Data
// ------------------------------
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg = "";
  for (int i = 0; i < len; i++) {
    msg += (char)incomingData[i];
  }

  Serial.print("📩 Received: ");
  Serial.println(msg);

  // Expected format: node_id,command,ack,message_id
  int idx1 = msg.indexOf(',');
  int idx2 = msg.indexOf(',', idx1 + 1);
  int idx3 = msg.indexOf(',', idx2 + 1);

  if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
    String node_id = msg.substring(0, idx1);
    String command = msg.substring(idx1 + 1, idx2);
    String type    = msg.substring(idx2 + 1, idx3);
    String msg_id  = msg.substring(idx3 + 1);

    if (type == "ack") {
      Serial.print("✅ ACK from ");
      Serial.print(node_id);
      Serial.print(": ");
      Serial.print(command);
      Serial.print(" (msg_id=");
      Serial.print(msg_id);
      Serial.println(")");
    } else {
      Serial.println("❌ Not an ACK type");
    }
  } else {
    Serial.println("❌ Invalid ACK format");
  }
}

// ------------------------------
// ⚙️ Setup
// ------------------------------
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);       // ESP-NOW requires STA or AP+STA mode
  WiFi.disconnect();         // Disconnect from any previous network

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }

  // Register broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    Serial.println("✅ Broadcast peer added.");
  }

  // Register receive callback
  esp_now_register_recv_cb(onReceive);

  Serial.println("✅ Gateway Ready. Type command as: node_id,command");
}

// ------------------------------
// 🔁 Loop: Send CMD via Serial Input
// ------------------------------
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int commaIndex = input.indexOf(',');
    if (commaIndex < 0) {
      Serial.println("⚠️ Format should be node_id,command");
      return;
    }

    // Extract node ID and command
    String node_id = input.substring(0, commaIndex);
    String command = input.substring(commaIndex + 1);

    // Construct message
    Message msg;
    msg.gw_id = gatewayID;
    msg.node_id = node_id;
    msg.command = command;
    msg.type = "cmd";
    msg.msg_id = generateMessageID();

    // Serialize message
    String payload = msg.gw_id + "," + msg.node_id + "," + msg.command + "," + msg.type + "," + msg.msg_id;
    
    // Send via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *)payload.c_str(), payload.length());

    // Print log
    Serial.println("📤 Sent CMD: " + payload);
  }
}
