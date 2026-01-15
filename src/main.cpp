// ✅ GATEWAY CODE (ESP-NOW CMD SENDER + ACK RECEIVER)
#include <config.h>

//Function prototypes
void networkTask(void *param); 
void mainTask(void *param);
void ledTask(void *param);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMqtt();
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
String generateMessageID();
bool isDuplicate(const String& msg_id);
void powerCycleModem();
bool initializeModem();
bool connectToNetwork();
void publishHeartbeat();
void publishData();

//Objects for GSM, MQTT, and ESP-NOW
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

//FreeRTOS Tasks and instance
QueueHandle_t ledQueue;
QueueHandle_t mqttQueue;
SemaphoreHandle_t modemMutex;
TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t ledTaskHandle;
TaskHandle_t otaTaskHandle = NULL;

// OTA control
volatile bool otaRequested = false;
String otaHost = otaHostDefault;
int otaPort = otaPortDefault;
String otaPath = otaPathDefault;
volatile bool otaInProgress = false;
//================================

// LDR to Lux conversion function
#ifdef USE_LDR_SENSOR
  float ldrToLux(int adc) {
    // Known calibration points
    const int ADC_vals[5] = {4048, 3800, 2096, 1966, 1600};
    const float Lux_vals[5] = {961, 488, 16.67, 11.67, 10.83};

    // If out of range
    if(adc >= ADC_vals[0]) return Lux_vals[0];
    if(adc <= ADC_vals[4]) return Lux_vals[4];

    // Find which segment
    for(int i=0; i<4; i++){
        if(adc <= ADC_vals[i] && adc >= ADC_vals[i+1]){
        float log_adc1 = log(ADC_vals[i]);
        float log_adc2 = log(ADC_vals[i+1]);
        float log_lux1 = log(Lux_vals[i]);
        float log_lux2 = log(Lux_vals[i+1]);

        float log_adc = log(adc);
        float log_lux = log_lux1 + (log_lux2 - log_lux1) * (log_adc - log_adc1) / (log_adc2 - log_adc1);

        return exp(log_lux);  // return interpolated Lux
        }
    }
    return 0; // fallback
  }
#endif
//==========================================================

// Function to publish heartbeat message
void publishHeartbeat(){
  MqttMessage hbMsg;
  snprintf(hbMsg.topic, MAX_TOPIC_LEN, MQTT_MC_HB);
  snprintf(hbMsg.payload, MAX_MQTT_MSG_LEN, "%s,wifi_connected", DEVICE_ID);

  xQueueSend(mqttQueue, &hbMsg, 0);
  
  LedBlink hbBlink = {CRGB::Blue, 500, 2, 300};  // on_duraton, repeat, gap_duration
  xQueueSend(ledQueue, &hbBlink, 0);
}
//========================================//

// Function to publish sensor data
void publishData(){
  
    float lux = -1;
    // Read LDR value and convert to Lux
    #ifdef USE_LDR_SENSOR
      Serial.print("LDR Value: ");
      int ldrValue = analogRead(LDR_PIN);
      Serial.print(ldrValue);
      Serial.println();

      lux = ldrToLux(ldrValue);
      lux = lux * 1.45; // Calibration factor
      Serial.print("Calculated Lux: ");
      Serial.print(lux, 2);
      Serial.println(" lx");
    #endif
    //========================================//

    // Get light level from BH1750
    #ifdef USE_GY30
      
      if (lightMeter.measurementReady()) {
        lux = lightMeter.readLightLevel();
        Serial.print("BH1750 Light Level: ");
        Serial.print(lux, 2);
        Serial.println(" lx");
      } else {
        Serial.println("BH1750 Measurement not ready");
      }
    #endif
    //========================================//

    // Prepare and send MQTT data message
    MqttMessage dataMsg;
    float temp = -1;
    float hum = -1;
    float ppm = -1;

    snprintf(dataMsg.topic, MAX_TOPIC_LEN, MQTT_MC_PUB);
    snprintf(dataMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s,%s,%s",
            DEVICE_ID,
            (temp >= 0) ? String(temp, 2).c_str() : "N/A",
            (hum >= 0) ? String(hum, 2).c_str() : "N/A",
            (ppm >= 0) ? String(ppm, 2).c_str() : "N/A",
            (lux >= 0) ? String(lux, 2).c_str() : "N/A"
      );
    xQueueSend(mqttQueue, &dataMsg, 0);

    LedBlink hbBlink = {CRGB::Green, 500, 2, 300};  // on_duraton, repeat, gap_duration
    xQueueSend(ledQueue, &hbBlink, 0);
}
//========================================//


// Function to connect to GSM network
// ---- Safe delay that feeds watchdog ----
void safeDelayMs(uint32_t ms) {
  uint32_t end = millis() + ms;
  while (millis() < end) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
//========================================//

// ---- Power cycle the modem safely ----
void powerCycleModem() {
  digitalWrite(MODEM_PWR, LOW);
  safeDelayMs(1200);  // simulate power key press
  digitalWrite(MODEM_PWR, HIGH);

  int waitTime = 0;
  while (!modem.testAT() && waitTime < 15000) {
    esp_task_wdt_reset();
    // vTaskDelay(pdMS_TO_TICKS(500));
    safeDelayMs(500);
    waitTime += 500;
  }
}
//========================================//

// ---- Initialize the modem ----
bool initializeModem() {
  powerCycleModem();

  Serial.println("[NET] Restarting modem...");
  if (!modem.restart()) {
    Serial.println("[NET] ❌ Modem restart failed");
    return false;
  }

  Serial.println("[NET] ✅ Modem restarted successfully");

  // modem.sendAT("+CFUN=0");
  // modem.waitResponse(3000L);

  // --- Step 1: Force GPRS detach and reattach ---
  Serial.println("[NET] Resetting GPRS attachment...");
  modem.sendAT("+CGATT=0");
  modem.waitResponse(3000L);
  modem.sendAT("+CGATT=1");
  modem.waitResponse(10000L);

  // --- Step 2: Disconnect any old GPRS session ---
  Serial.println("[NET] Disconnecting previous GPRS session...");
  modem.gprsDisconnect();
  delay(1000);
  return true;
}
//========================================//

// ---- Connect to GSM + GPRS network safely ----
bool connectToNetwork() {
  Serial.println("[NET] Connecting to network...");
  leds[0] = CRGB::Red;  // Indicate GSM connecting
  FastLED.show();
  ledState = false;

  // Wait for GSM network registration
  unsigned long start = millis();
  while (!modem.waitForNetwork(1000)) {  // 1s wait per try
    esp_task_wdt_reset();
    if (millis() - start > 30000) {  // 30s timeout
      Serial.println("[NET] ❌ Network connection failed");
      gsmConnected = false;
      return false;
    }
    // vTaskDelay(pdMS_TO_TICKS(500));
    safeDelayMs(500);
  }

  // Try to connect to GPRS
  start = millis();
  while (!modem.gprsConnect(apn, apnUser, apnPass)) {
    esp_task_wdt_reset();
    if (millis() - start > 20000) {  // 20s timeout
      Serial.println("[NET] ❌ GPRS connection failed");
      gsmConnected = false;
      return false;
    }
    // vTaskDelay(pdMS_TO_TICKS(500));
    safeDelayMs(500);
  }

  Serial.println("[NET] ✅ Network connected");
  gsmConnected = true;
  return true;
}
//========================================//

// ---- MQTT state text helper ----
String mqttStateToText(int state) {
  switch (state) {
    case 0: return "Connected";
    case -1: return "Connection Timeout";
    case -2: return "Connection Lost";
    case -3: return "Connect Failed";
    case -4: return "Disconnected";
    case -5: return "Bad Protocol";
    case -6: return "Bad Client ID";
    case -7: return "Unavailable";
    case -8: return "Bad Credentials";
    case -9: return "Unauthorized";
    default: return "Unknown";
  }
}
//========================================//

// ---- MQTT reconnect with exponential backoff and ESP32 restart ----
void reconnectMqtt() {
    static uint8_t mqttFailCount = 0;
    static unsigned long lastAttempt = 0;
    static unsigned long retryInterval = 1000; // start 1 sec
    const unsigned long MAX_INTERVAL = 60000;
    const uint8_t MAX_FAILS_BEFORE_RESTART = 20;

    if (!mqtt.connected() && modem.isGprsConnected()) {
        unsigned long now = millis();
        if (now - lastAttempt < retryInterval) return;
        lastAttempt = now;

        char clientId[32];
        snprintf(clientId, sizeof(clientId), "MeshAC_%04X%04X%04X",
                  random(0xFFFF), random(0xFFFF), random(0xFFFF));

        Serial.printf("[%lu ms] [MQTT] Connecting as client ID: %s\n", millis(), clientId);

        if (mqtt.connect(clientId, mqttUser, mqttPass)) {
            mqttFailCount = 0;
            retryInterval = 5000; // reset backoff
            Serial.println("[MQTT] ✅ Connected");

            snprintf(mqttSubTopic, sizeof(mqttSubTopic), "%s/%s", MQTT_MC_SUB, DEVICE_ID);
            mqtt.subscribe(mqttSubTopic);

            leds[0] = CRGB::Green; // indicate connected
            FastLED.show();
        } else {
            esp_task_wdt_reset();
            mqttFailCount++;
            retryInterval = min(retryInterval * 2, MAX_INTERVAL); // exponential backoff
            Serial.printf("[MQTT] ❌ Connect failed (%d: %s) | Retry %d | Next attempt in %lus\n",
                          mqtt.state(),
                          mqttStateToText(mqtt.state()).c_str(),
                          mqttFailCount,
                          retryInterval / 1000);

            leds[0] = CRGB::Yellow;
            FastLED.show();

            if (mqttFailCount >= MAX_FAILS_BEFORE_RESTART) {
                Serial.println("[MQTT] ⚠️ Too many failures — restarting ESP32...");
                delay(2000);
                ESP.restart();
            }
        }
    }
}
//========================================//

// Callback function for MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // LedBlink mqttbBlink = {CRGB::Blue, 150, 1, 150};  // on_duraton, repeat, gap_duration
  // xQueueSend(ledQueue, &mqttbBlink, 0);

  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("[MQTT IN] Topic: " + String(topic));
  Serial.println("[MQTT IN] Message: " + message);

  message.trim();           // Removes leading/trailing whitespace
  message.replace(" ", ""); // Removes all internal spaces
  Serial.println("📥 Message: " + message);
  //=======================================
  LedBlink mqttMsgBlink = {CRGB::Blue, 250, 1, 250};  // on_duraton, repeat, gap_duration
  xQueueSend(ledQueue, &mqttMsgBlink, 0);

  String m = message;
  if (m == "update_firmware") {
    // Optional: parse payload for host/path/port (very minimal parsing)
    // Expect payload like: ota;http://host:port/path or {"host":"x","port":5000,"path":"/firm.bin"}
    // We'll do a simple check for "http://" or a JSON parse fallback.
    if (m.indexOf("http://") >= 0 || m.indexOf("https://") >= 0) {
        // Very basic parse: extract host, optional :port, and path
        int start = m.indexOf("http://");
        if (start < 0) start = m.indexOf("https://");
        String url = message.substring(start);
        // remove protocol
        if (url.startsWith("http://")) url = url.substring(7);
        else if (url.startsWith("https://")) url = url.substring(8);
        int slash = url.indexOf('/');
        String hostport = (slash >= 0) ? url.substring(0, slash) : url;
        String path = (slash >= 0) ? url.substring(slash) : "/";
        int colon = hostport.indexOf(':');
        if (colon >= 0) {
            otaHost = hostport.substring(0, colon);
            otaPort = hostport.substring(colon + 1).toInt();
        } else {
            otaHost = hostport;
            otaPort = 80;
        }
        otaPath = path;
    } else {
        // Minimal JSON-ish parse: look for "path":"...". Not robust — adjust per your payload format.
        int pIndex = message.indexOf("path");
        if (pIndex >= 0) {
            int quote1 = message.indexOf('"', pIndex);
            int quote2 = message.indexOf('"', quote1 + 1);
            int quote3 = message.indexOf('"', quote2 + 1);
            int quote4 = message.indexOf('"', quote3 + 1);
            if (quote3 >= 0 && quote4 > quote3) {
                otaPath = message.substring(quote3 + 1, quote4);
            }
        }
        // if no host provided, use defaults
        otaHost = otaHostDefault;
        otaPort = otaPortDefault;
    }

    Serial.printf("[OTA] Request queued: host=%s port=%d path=%s\n", otaHost.c_str(), otaPort, otaPath.c_str());
    otaRequested = true;
    
    LedBlink otaBlink = {CRGB::LightGreen, 250, 2, 250};  // on_duraton, repeat, gap_duration
    xQueueSend(ledQueue, &otaBlink, 0);
    delay(1000);
    return;
}


  // Check if the message is a ping
  if(message == "ping") {
    Serial.println("Ping Arrived!");
    // Send back an ACK
    MqttMessage mqttMsg;
    snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_MC_PUB);
    snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,gsm_available", DEVICE_ID);
    xQueueSend(mqttQueue, &mqttMsg, 0);
    
    LedBlink pingBlink = {CRGB::Green, 250, 2, 250};  // on_duraton, repeat, gap_duration
    xQueueSend(ledQueue, &pingBlink, 0);
    return;
  }
  //====================================

  int commaIndex = message.indexOf(',');
  if (commaIndex < 0) {
    Serial.println("⚠️ Format: node_id,command");
    LedBlink cmdErrorBlink = {CRGB::Orange, 150, 2, 150};  // on_duraton, repeat, gap_duration
    xQueueSend(ledQueue, &cmdErrorBlink, 0);
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
//==========================================================//

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
//========================================//

// Function to generate a unique message ID
String generateMessageID() {
  uint16_t randNum = esp_random() & 0xFFFF;
  // Serial.print("Raw 16-bit randNum: ");
  // Serial.println(randNum);
  char id[5];
  sprintf(id, "%04X", randNum);
  return String(id);
}
//========================================//

// Callback function for receiving ESP-NOW messages
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  String msg((char*)incomingData, len);
  DEBUG_PRINTLN("\n📥 Received: " + msg);

  int commaCount = std::count(msg.begin(), msg.end(), ',');
  if (commaCount != 4) {
    DEBUG_PRINTLN("❌ Invalid message format. Skipped.");
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
    DEBUG_PRINTLN("⚠️ Duplicate " + type + " ignored (id=" + msg_id + ")");
    return;
  }

  // Only process known types
  if (type != "smswt_hb" && type != "smswt_ack") {
    DEBUG_PRINTLN("⏭ Ignored unknown type: " + type);
    return;
  }

  #if DEBUG_MODE
    Serial.printf("✅ %s Received: sender=%s → receiver=%s | cmd=%s | id=%s\n",
                  type.c_str(), sender_id.c_str(), receiver_id.c_str(),
                  command.c_str(), msg_id.c_str());
  #endif

  // Prepare MQTT message
  MqttMessage mqttMsg;


  //Smart Switch MQTT Topics
  if (type == "smswt_hb"){
    snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_SMARTSWITCH_HB);
    snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, sender_id.c_str(), command.c_str());
  }
  else if (type == "smswt_ack"){
    snprintf(mqttMsg.topic, MAX_TOPIC_LEN, MQTT_SMARTSWITCH_ACK);
    snprintf(mqttMsg.payload, MAX_MQTT_MSG_LEN, "%s,%s,%s", DEVICE_ID, sender_id.c_str(), command.c_str());

    LedBlink ackBlink = {CRGB::Green, 150, 1, 150};  // on_duraton, repeat, gap_duration
    xQueueSend(ledQueue, &ackBlink, 0);
  }

  xQueueSend(mqttQueue, &mqttMsg, 0);

  // Re-broadcast if needed
  // rebroadcastIfNeeded(msg_id, type, msg);
}
//========================================//


// =============================
// NETWORK TASK CORE
// =============================
void networkTask(void *param) {
  // esp_task_wdt_add(NULL);
  leds[0] = CRGB::Red;  // GSM not connected
  FastLED.show();
  ledState = false;

  uint8_t connectionRetries = 0;
  const uint8_t MAX_CONNECTION_RETRIES = 5;
  const unsigned long CONNECTION_RETRY_DELAY = 30000UL;

  unsigned long lastNetworkCheck = 0;
  const unsigned long NETWORK_CHECK_INTERVAL = 60000; // 30 seconds

  Serial.println("[NET] Network task started");

  // ----- INITIALIZE MODEM + CONNECT -----
  if (!initializeModem() || !connectToNetwork()) {
    Serial.println("[NET] Initialization failed - restarting ESP");
    safeDelayMs(1000);
    ESP.restart();
  }

  unsigned long lastReconnectAttempt = 0;
  MqttMessage msg;

  for (;;) {
    // esp_task_wdt_reset();  // Feed watchdog

    // If OTA requested, start shutdown & create otaTask
    if (otaRequested && !otaInProgress) {
        otaInProgress = true;
        Serial.println("[NET] OTA requested - preparing for OTA");

        leds[0] = CRGB::White;
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(50));

        // Publish ack
        mqtt.publish(MQTT_OTA_PUB, "OTA_Started");

        // Stop MQTT gracefully
        mqtt.disconnect();

        // Stop GPRS data or keep modem powered for OTA (we need client)
        // We'll keep modem powering, and reuse gsmClient in otaTask

        // Remove WDT registrations for tasks we will delete
        if (mainTaskHandle) {
            esp_task_wdt_delete(mainTaskHandle);
        }
        
        if (ledTaskHandle) {
            esp_task_wdt_delete(ledTaskHandle);
        }
        
        // delete mainTask
        if (mainTaskHandle) {
            vTaskDelete(mainTaskHandle);
            mainTaskHandle = NULL;
            Serial.println("[NET] mainTask deleted");
        }
        
        if (ledTaskHandle) {
            vTaskDelete(ledTaskHandle);
            mainTaskHandle = NULL;
            Serial.println("[NET] ledTask deleted");
        }

        // Create otaTask pinned to core 1 (or 0) - give it a larger stack
        BaseType_t created = xTaskCreatePinnedToCore(
            [](void*)->void {
                // stub (real function defined below). This lambda placeholder shouldn't be used.
                // We'll never call this lambda; create below with otaTask function pointer.
                vTaskDelete(NULL);
            },
            "otaTask", OTA_TASK_STACK_SIZE, NULL, NETWORK_TASK_PRIORITY, &otaTaskHandle, 0
        );
        // The above lambda is just a placeholder to reserve handle; we'll delete it and create proper one
        if (created == pdPASS) {
            vTaskDelete(otaTaskHandle); // remove placeholder
            otaTaskHandle = NULL;
        }

        // Proper creation using otaTask function below
        extern void otaTask(void* pv);
        if (xTaskCreatePinnedToCore(otaTask, "otaTask", OTA_TASK_STACK_SIZE, NULL, NETWORK_TASK_PRIORITY, &otaTaskHandle, 1) == pdPASS) {
            Serial.println("[NET] otaTask created");
        } else {
            Serial.println("[NET] otaTask creation FAILED");
            // Try to recover: reboot
            ESP.restart();
        }

        // Remove networkTask from WDT and delete self (networkTask) to let otaTask take over
        esp_task_wdt_delete(NULL);
        Serial.println("[NET] networkTask self-deleting to hand control to otaTask");
        vTaskDelete(NULL); // delete network task (this function returns no further)
    }
    // =====================
    
    // GSM/GPRS CONNECTION MANAGEMENT
    // =====================
    if (!modem.isGprsConnected()) {
      Serial.println("[NET] ⚠ GPRS disconnected");

      if (!connectToNetwork()) {
        // Retry logic
        if (connectionRetries >= MAX_CONNECTION_RETRIES) {
          Serial.println("[NET] Max retries reached → Resetting modem...");
          initializeModem();
          connectionRetries = 0;
        } else {
          Serial.printf("[NET] Retry %d/%d in %lus...\n",
                        connectionRetries + 1,
                        MAX_CONNECTION_RETRIES,
                        CONNECTION_RETRY_DELAY / 1000);

          // Non-blocking wait with WDT feeding
          uint32_t retryStart = millis();
          while (millis() - retryStart < CONNECTION_RETRY_DELAY) {
            // esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
          }

          connectionRetries++;
        }
        continue;  // Skip rest until connected
      }
      connectionRetries = 0;  // Reset on success
    }

    // =====================
    // MQTT CONNECTION MANAGEMENT
    // =====================
    if (modem.isGprsConnected() && !mqtt.connected()) {
      leds[0] = CRGB::Yellow;  // MQTT not connected
      FastLED.show();
      ledState = false;
      reconnectMqtt();
    }

    // =====================
    // MQTT LOOP + PUBLISH
    // =====================
    if (mqtt.connected()) {
      if (!ledState) {
        leds[0] = CRGB::Black;  // All good
        FastLED.show();
        ledState = true;
      }

      mqtt.loop();  // process inbound/outbound packets
      // esp_task_wdt_reset();

      // Check for any message to publish
      if (xQueueReceive(mqttQueue, &msg, 0) == pdTRUE) {
        if (mqtt.publish(msg.topic, msg.payload)) {
          Serial.printf("[MQTT] Published: %s → %s\n", msg.topic, msg.payload);
        } else {
          Serial.printf("[MQTT] ❌ Publish failed: %s\n", msg.topic);
          LedBlink dataSendErrorBlink = {CRGB::Red, 250, 2, 250};
          xQueueSend(ledQueue, &dataSendErrorBlink, 0);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(250));  // Yield CPU, keep loop responsive
  }
}
//================================================================//

// =============================
// MAIN TASK CORE
// Main task to handle serial commands, heartbeat, and Modbus data
void mainTask(void *param) {
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    // 📥 Serial command handler
    /*
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
    */

    // 💓 Heartbeat via MQTT queue
    if (millis() - lastHBPublishTime >= hbPublishInterval) {
    lastHBPublishTime = millis();
    //-----------------------------------//

    // Publish heartbeat
    publishHeartbeat();

    
  }

  //Data sending vial mqttQueue can be added here
  if(millis() - lastDataPublishTime >= dataPublishInterval) {
    lastDataPublishTime = millis();
    //-----------------------------------//

    // Publish data
    publishData();

  }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Yield for watchdog
  }
}
//=====================================================================//

//LED Task to handle LED blinking based on messages from ledQueue
void ledTask(void *param) {
    esp_task_wdt_add(NULL);  // register with WDT
    LedBlink blink;

    for (;;) {
        // Wait for a new blink message, but don't block forever
        if (xQueueReceive(ledQueue, &blink, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < blink.repeat; i++) {
                leds[0] = blink.color;
                FastLED.show();
                safeDelayMs(blink.duration);   // feed WDT inside safeDelayMs

                leds[0] = CRGB::Black;
                FastLED.show();

                if (i < blink.repeat - 1) {
                    safeDelayMs(blink.gap);  // feed WDT inside safeDelayMs
                }
            }
        } else {
            // Queue empty, just feed WDT and yield
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
//==================================================//

// OTA Task to handle over-the-air firmware updates
void otaTask(void* pvParameters) {
    Serial.println("[OTA] otaTask started");
    // We will not register otaTask to WDT to avoid unwanted resets during long flash.
    // If WDT enabled globally, ensure it won't kill this task:
    // (We deliberately don't call esp_task_wdt_add for this task.)

    // Ensure modem & GPRS connected (if networkTask gracefully kept modem up, this should be fine)
    if (!modem.isGprsConnected()) {
        Serial.println("[OTA] GPRS not connected, trying to connect...");
        if (!connectToNetwork()) {
            Serial.println("[OTA] Failed to connect to network - aborting OTA");
            mqtt.publish(MQTT_OTA_PUB, "OTA_Failed_to_Connect");
            otaRequested = false;
            otaInProgress = false;
            // vTaskDelay(pdMS_TO_TICKS(2000));
            safeDelayMs(2000);
            ESP.restart();
        }
    }

    // Connect to OTA server
    Serial.printf("[OTA] Connecting to %s:%d\n", otaHost.c_str(), otaPort);
    if (!gsmClient.connect(otaHost.c_str(), otaPort)) {
        Serial.println("[OTA] gsmClient.connect failed!");
        // attempt to report via serial and reboot
        mqtt.publish(MQTT_OTA_PUB, "OTA_Failed_to_Connect_Server");
        otaRequested = false;
        otaInProgress = false;
        // vTaskDelay(pdMS_TO_TICKS(2000));
        safeDelayMs(2000);
        ESP.restart();
    }

    // Send HTTP GET
    String getReq = String("GET ") + otaPath + " HTTP/1.1\r\n" +
                    "Host: " + otaHost + "\r\n" +
                    "Connection: close\r\n\r\n";
    gsmClient.print(getReq);
    Serial.println("[OTA] HTTP GET sent, waiting for response...");

    // wait for headers
    unsigned long start = millis();
    while (!gsmClient.available() && (millis() - start) < 10000) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!gsmClient.available()) {
        Serial.println("[OTA] No response from server");
        mqtt.publish(MQTT_OTA_PUB, "OTA_No_Response_from_Server");
        otaRequested = false;
        otaInProgress = false;
        gsmClient.stop();
        ESP.restart();
    }

    // Parse HTTP response headers
    int contentLength = -1;
    bool isChunked = false;
    bool httpOk = false;
    while (gsmClient.available()) {
        String line = gsmClient.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) break; // end headers
        line.toLowerCase();
        if (line.startsWith("http/1.1 200") || line.startsWith("http/1.0 200")) httpOk = true;
        if (line.startsWith("content-length:")) {
            contentLength = line.substring(15).toInt();
            contentLength = atoi(line.substring(15).c_str());
        }
        if (line.startsWith("transfer-encoding:") && line.indexOf("chunked") >= 0) {
            isChunked = true;
        }
    }

    if (!httpOk) {
        Serial.println("[OTA] HTTP not OK");
        mqtt.publish(MQTT_OTA_PUB, "OTA_HTTP_Not_OK");
        otaRequested = false;
        otaInProgress = false;
        gsmClient.stop();
        ESP.restart();
    }

    if (isChunked) {
        Serial.println("[OTA] Chunked transfer- not supported by this simple OTA (abort).");
        mqtt.publish(MQTT_OTA_PUB, "OTA_Chunked_Not_Supported");
        otaRequested = false;
        otaInProgress = false;
        gsmClient.stop();
        ESP.restart();
    }

    if (contentLength <= 0) {
        Serial.println("[OTA] Invalid content length");
        mqtt.publish(MQTT_OTA_PUB, "OTA_Invalid_Content_Length");
        otaRequested = false;
        otaInProgress = false;
        gsmClient.stop();
        ESP.restart();
    }

    Serial.printf("[OTA] Content-Length: %d\n", contentLength);

    // Start Update
    if (!Update.begin(contentLength)) {
        Serial.printf("[OTA] Update.begin failed: %s\n", Update.errorString());
        mqtt.publish(MQTT_OTA_PUB, "OTA_Update_Begin_Failed");
        otaRequested = false;
        otaInProgress = false;
        gsmClient.stop();
        ESP.restart();
    }

    // Read body and write to flash
    uint8_t buf[1024];
    int totalRead = 0;
    while (totalRead < contentLength) {
        int toRead = sizeof(buf);
        if (contentLength - totalRead < toRead) toRead = contentLength - totalRead;
        int r = gsmClient.readBytes(buf, toRead);
        if (r <= 0) {
            // wait a bit for more data (but not forever)
            int waitCount = 0;
            while (gsmClient.available() == 0 && waitCount++ < 50) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (gsmClient.available() == 0) {
                Serial.println("[OTA] Read timeout");
                Update.abort();
                mqtt.publish(MQTT_OTA_PUB, "OTA_Read_Timeout");
                otaRequested = false;
                otaInProgress = false;
                gsmClient.stop();
                ESP.restart();
            } else continue;
        }
        size_t written = Update.write(buf, r);
        if (written != (size_t)r) {
            Serial.printf("[OTA] Write mismatch: wrote %u expected %d\n", (unsigned)written, r);
            Update.abort();
            mqtt.publish(MQTT_OTA_PUB, "OTA_Write_Mismatch");
            otaRequested = false;
            otaInProgress = false;
            gsmClient.stop();
            ESP.restart();
        }
        totalRead += r;
        float pct = (100.0 * totalRead) / contentLength;
        Serial.printf("\r[OTA] Progress: %.1f%%", pct);
    }
    Serial.println();

    // Finalize
    if (!Update.end()) {
        Serial.printf("[OTA] Update.end failed: %s\n", Update.errorString());
        mqtt.publish(MQTT_OTA_PUB, "OTA_Update_End_Failed");
        otaRequested = false;
        otaInProgress = false;
        gsmClient.stop();
        ESP.restart();
    }

    if (!Update.isFinished()) {
        Serial.println("[OTA] Update not finished");
        mqtt.publish(MQTT_OTA_PUB, "OTA_Update_Not_Finished");
        otaRequested = false;
        otaInProgress = false;
        gsmClient.stop();
        ESP.restart();
    }

    Serial.println("[OTA] Update successful - restarting...");
    mqtt.publish(MQTT_OTA_PUB, "OTA_Update_Successful!!! Restarting");
    delay(1500);
    ESP.restart();

    // Should never reach here
    vTaskDelete(NULL);
}
// ================================================================

// Function to check if it's the top of the hour
void setup() {
  Serial.begin(115200);

  preferences.begin("device_data", false);  // Open Preferences (NVS)
  static String device_id; // Static variable to persist scope
  
  //Device ID Management
  #if CHANGE_DEICE_ID
    // Construct new device ID
    device_id = String(WORK_PACKAGE) + GW_TYPE + FIRMWARE_UPDATE_DATE + DEVICE_SERIAL;
    
    // Save device ID to Preferences
    preferences.putString("device_id", device_id);
    Serial.println("Device ID updated in Preferences: " + device_id);
  #else
    // Restore device ID from Preferences
    device_id = preferences.getString("device_id", "UNKNOWN");
    Serial.println("Restored Device ID from Preferences: " + device_id);
  #endif
  //=========================================//
  
  DEVICE_ID = device_id.c_str(); // Assign to global pointer
  
  preferences.end();

  //=========================================
  
  // Serial.println("Gateway ID: " + String(DEVICE_ID));

  #ifdef USE_FastLED
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
  #endif

  Serial.println("🔄 Starting Gateway...");
  DEBUG_PRINT("Device ID: ");
  DEBUG_PRINTLN(DEVICE_ID);

  // ---- LDR SENSOR SETUP ----
  #ifdef USE_LDR_SENSOR
    Serial.println("🔄 Initializing LDR Sensor...");
    pinMode(LDR_PIN, INPUT);
    Serial.println("\n✅ LDR Sensor Test");
  #endif

  #ifdef USE_GY30
    // Initialize I2C
    Wire.begin();
    // Initialize BH1750 (default address 0x23)
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        Serial.println("✅BH1750 initialized successfully");
    } else {
        Serial.println("❌Error initializing BH1750");
    }
  #endif
  //========================================//

  SerialAT.begin(SIM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(MODEM_PWR, OUTPUT);
  digitalWrite(MODEM_PWR, HIGH);

  mqtt.setServer(broker, MQTT_PORT);
  mqtt.setKeepAlive(60);
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

  mqttQueue = xQueueCreate(50, sizeof(MqttMessage));
  if (mqttQueue == NULL) {
    Serial.println("❌ Failed to create mqttQueue");
    while (1); // Stop here if failed
  }

  ledQueue = xQueueCreate(10, sizeof(LedBlink));
  if (ledQueue == NULL) {
    Serial.println("❌ Failed to create ledQueue");
    while (true); // Stop here if failed
  }

    // Initialize WDT for all tasks
  esp_task_wdt_init(60, true);   // 🛡️ 60s timeout for all registered tasks 
  Serial.println("✅ WDT Initialized");

  Serial.println("✅ Gateway Ready to Works!");

  xTaskCreatePinnedToCore(ledTask, "LED Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(networkTask, "Network Task", 8 * 1024, NULL, NETWORK_TASK_PRIORITY, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 8 * 1024, NULL, 1, &mainTaskHandle, 1);

}

// Function to check if it's the top of the hour
void loop() {
  vTaskDelay(pdMS_TO_TICKS(100)); 
}
