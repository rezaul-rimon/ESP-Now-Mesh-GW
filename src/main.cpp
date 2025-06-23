// ✅ GATEWAY CODE (ESP-NOW CMD SENDER + ACK RECEIVER)
#include <config.h>

//Function prototypes
void networkTask(void *param); 
void mainTask(void *param);
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool connectGSM();
void reconnectMqtt();
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
String generateMessageID();
bool isDuplicate(const String& msg_id);

//Objects for GSM, MQTT, and ESP-NOW
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

//FreeRTOS Tasks and instance
QueueHandle_t mqttQueue;
SemaphoreHandle_t modemMutex;
TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
// TaskHandle_t wifiResetTaskHandle;

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
  msg.sender_id = Local_ID;
  msg.receiver_id = message.substring(0, commaIndex);
  msg.command = message.substring(commaIndex + 1);
  msg.type = "cmd";
  msg.msg_id = generateMessageID();

  String payload2 = msg.sender_id + "," + msg.receiver_id + "," + msg.command + "," + msg.type + "," + msg.msg_id;
  esp_now_send(broadcastAddress, (uint8_t*)payload2.c_str(), payload2.length());
  Serial.println("📤 CMD Sent: " + payload2);

  // You can add command handling here if needed
}

// Function to check for duplicate ACKs
bool isDuplicate(const String& type, const String& msg_id) {
  String key = type + ":" + msg_id;
  if (std::find(recentMsgKeys.begin(), recentMsgKeys.end(), key) != recentMsgKeys.end()) {
    return true;
  }
  recentMsgKeys.push_back(key);
  if (recentMsgKeys.size() > maxRecentIDs) {
    recentMsgKeys.pop_front();
  }
  return false;
}

// Function to generate a unique message ID
String generateMessageID() {
  uint16_t randNum = esp_random() & 0xFFFF;
  // Serial.print("Raw 16-bit randNum: ");
  // Serial.println(randNum);
  char id[5];
  sprintf(id, "%04X", randNum);
  return String(id);
}

// Callback function for receiving ESP-NOW messages
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg((char*)incomingData, len);
  Serial.println("\n📥 Received: " + msg);

  int commaCount = std::count(msg.begin(), msg.end(), ',');
  if (commaCount != 4) {
    Serial.println("❌ Invalid message format. Skipped.");
    return;
  }

  int idx1 = msg.indexOf(',');
  int idx2 = msg.indexOf(',', idx1 + 1);
  int idx3 = msg.indexOf(',', idx2 + 1);
  int idx4 = msg.indexOf(',', idx3 + 1);

  String sender_id   = msg.substring(0, idx1);
  String receiver_id = msg.substring(idx1 + 1, idx2);
  String command     = msg.substring(idx2 + 1, idx3);
  String type        = msg.substring(idx3 + 1, idx4);
  String msg_id      = msg.substring(idx4 + 1);
  
  // 🔁 Deduplication for ALL types
  if (isDuplicate(type, msg_id)) {
    Serial.println("⚠️ Duplicate " + type + " ignored (id=" + msg_id + ")");
    return;
  }

  // Only process known types
  if (type != "ack" && type != "hb" && type != "tmp") {
    Serial.println("⏭ Ignored unknown type: " + type);
    return;
  }

  Serial.printf("✅ %s Received: sender=%s → receiver=%s | cmd=%s | id=%s\n",
                type.c_str(), sender_id.c_str(), receiver_id.c_str(),
                command.c_str(), msg_id.c_str());

  // Prepare MQTT message
  MqttMessage mqttMsg;
  if (type == "ack") {
    snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_AC_ACK);
    snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, sender_id.c_str(), command.c_str());
  } 
  // else if (type == "hb" || type == "tmp") {
  //   snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_AC_HB);
  //   snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, sender_id.c_str(), command.c_str());
  // }
  else if (type == "hb") {
    snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_AC_HB);
    snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, sender_id.c_str(), command.c_str());
  } 
  else if (type == "tmp") {
    snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_AC_TMP);
    snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, sender_id.c_str(), command.c_str());
  }

  xQueueSend(mqttQueue, &mqttMsg, 0);

  // Re-broadcast if needed
  // rebroadcastIfNeeded(msg_id, type, msg);
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

// Function to parse Modbus data and prepare MQTT message
void ParsingModbusData() {
  // Combine 32-bit energy register from taeHigh and taeLow
  uint32_t totalEnergyRaw = ((uint32_t)taeHigh << 16) | (uint32_t)taeLow;

  // Scale energy — assuming it's in 0.1 kWh units
  float totaltNetEnergy = totalEnergyRaw * 0.1;
  float tImpEnergy = totalEnergyRaw * 0.1;

  // Apply proper scaling
  float ap = activePower * 0.1;
  float va = pAvolt * 0.1;
  float vb = pBvolt * 0.1;
  float vc = pCvolt * 0.1;
  float vab = lABvolt * 0.1;
  float vbc = lBCvolt * 0.1;
  float vca = lCAvolt * 0.1;
  float ia = pAcurrent * 0.1;
  float ib = pBcurrent * 0.1;
  float ic = pCcurrent * 0.1;
  float freq = frequency * 0.01;
  float pf = powerFactor * 0.001;

  // Optional debug prints to verify scaling
  Serial.println("---- Scaled Values ----");
  Serial.printf("Energy: %.2f kWh, Power: %.2f W\n", totaltNetEnergy, ap);
  Serial.printf("Voltages: VA=%.2f, VB=%.2f, VC=%.2f, VAB=%.2f, VBC=%.2f, VCA=%.2f\n", va, vb, vc, vab, vbc, vca);
  Serial.printf("Currents: IA=%.2f, IB=%.2f, IC=%.2f\n", ia, ib, ic);
  Serial.printf("Frequency: %.2f Hz, PF: %.3f\n", freq, pf);

  // Format the final MQTT message string
  snprintf(em_data, sizeof(em_data),
    "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
    DEVICE_ID,
    totaltNetEnergy, tImpEnergy, ap,
    va, vb, vc,
    vab, vbc, vca,
    ia, ib, ic,
    freq, pf);
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
        Serial.println("⚠️ Format: receiver_id,command");
      } else {
        Message msg;
        msg.sender_id = Local_ID;
        msg.receiver_id = input.substring(0, commaIndex);
        msg.command = input.substring(commaIndex + 1);
        msg.type = "cmd";
        msg.msg_id = generateMessageID();

        String payload = msg.sender_id + "," + msg.receiver_id + "," + msg.command + "," + msg.type + "," + msg.msg_id;
        esp_now_send(broadcastAddress, (uint8_t*)payload.c_str(), payload.length());
        Serial.println("📤 CMD Sent: " + payload);
      }
    }

    // 💓 Heartbeat via MQTT queue
    if (millis() - lastHBPublishTime >= hbPublishInterval) {
    lastHBPublishTime = millis();
    bool acLineState = digitalRead(AC_LINE_PIN);

    MqttMessage hbMsg;
    snprintf(hbMsg.topic, MAX_TOPIC_LEN, MQTT_EM_HB);
    snprintf(hbMsg.payload, MAX_MQTT_MSG_LEN, "%s,W:0,G:1,C:%d,SD:%d",
      DEVICE_ID,
      acLineState ? 1 : 0,
      USE_SD_CARD ? 1 : 0);

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
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Yield for watchdog
  }
}

// Data Publish Task
void mqttPublishTask(void *param) {
  MqttMessage msg;
  for (;;) {
    if (mqtt.connected()) {
      if (xQueueReceive(mqttQueue, &msg, portMAX_DELAY) == pdTRUE) {
        if (xSemaphoreTake(modemMutex, pdMS_TO_TICKS(1000))) {
          if (mqtt.publish(msg.topic, msg.payload)) {
            Serial.printf("[MQTT] Published to %s: %s\n", msg.topic, msg.payload);
            // LED logic...
          } else {
            Serial.printf("[MQTT] Failed to publish to %s\n", msg.topic);
          }
          xSemaphoreGive(modemMutex);
        } else {
          Serial.println("⚠️ Could not acquire modem mutex in mqttPublishTask");
          xQueueSendToFront(mqttQueue, &msg, 0); // Requeue to not lose message
        }
      }
    } else {
      // Wait and retry after a short delay
      vTaskDelay(pdMS_TO_TICKS(1000));
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

  Serial.println("Gateway ID: " + String(DEVICE_ID));


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

  Serial.println("✅ Gateway Ready to Works!");

  xTaskCreatePinnedToCore(networkTask, "Network Task", 8 * 1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16 * 1024, NULL, 1, &mainTaskHandle, 1);
  xTaskCreatePinnedToCore(mqttPublishTask, "MQTT Pub Task", 6 * 1024, NULL, 1, NULL, 1);

}

// Function to check if it's the top of the hour
void loop() {
  
}
