// ================================================================
// ESP32 IoT Firmware
// ================================================================

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <ModbusMaster.h>
#include <SD.h>
#include <SPI.h>
#include <FS.h>

// ================================================================
// CONSTANTS & CONFIGURATION
// ================================================================
#define SERVICE_UUID        "204fefb3-3d9b-4e3f-8f76-8245e29ac6e9"
#define CHAR_UUID_WRITE     "c639bc5a-c5fa-48e4-814b-257a2cfc425e"
#define CHAR_UUID_NOTIFY    "63b05182-23a1-43e7-855b-a85cf8f7b7fb"

#define RX1_PIN 18
#define TX1_PIN 17
#define SD_CS_PIN    9   // GP9
#define SD_MOSI_PIN 11   // GP11
#define SD_SCK_PIN  12   // GP12
#define SD_MISO_PIN 13   // GP13

const int SENSOR_TYPE = 0; // 0 = DPT, 1 = RHT

// Timing Constants
const unsigned long WATCHDOG_TIMEOUT = 3000; 
const unsigned long FILE_CHECK_INTERVAL = 900000; // 15 minutes
const unsigned long WIFI_RECONNECT_INTERVAL = 60000; // 1 minute
const unsigned long SD_OPERATION_TIMEOUT = 5000; // 5 seconds
const unsigned long HTTP_TIMEOUT = 5000; // 5 seconds
const int WIFI_CONNECT_ATTEMPTS = 20; 
const int MAX_WIFI_NETWORKS_SAVED = 5;
const int MAX_SCAN_RESULTS = 15;

// Validation Constants
const int MIN_UPDATE_INTERVAL = 1;
const int MAX_UPDATE_INTERVAL = 86400; // 24 hours
const float MIN_SETPOINT = -9999.0;
const float MAX_SETPOINT = 9999.0;

// ================================================================
// GLOBAL OBJECTS
// ================================================================
Preferences preferences;
NimBLECharacteristic* pNotifyCharacteristic = nullptr;
ModbusMaster modbus;

SPIClass sdSPI(HSPI);
bool sdReady = false;

// ================================================================
// STATE VARIABLES
// ================================================================
bool deviceConnected = false;
bool triggerWifiScan = false;
bool wifiConfigReceived = false;
bool watchdogPaused = false;
bool forceHttpNow = false;

float setPoint1 = 0.0;
float setPoint2 = 0.0;

unsigned long lastFileCheckTime = 0;
unsigned long lastHttpTime = 0;
unsigned long lastWatchdogTime = 0;
unsigned long lastWifiCheck = 0;
int lastClockMinute = -1;

String targetSSID = "";
String targetPass = "";

String DEVICE_NICKNAME = ""; 
String defaultName = "";

// Config Variables
String DEVICE_ID = "ESP_001";
String API_URL = "https://cloudbases.in/iot_demo24/Api";
String NTP_SERVER = "1.in.pool.ntp.org";
int UPDATE_INTERVAL = 60;
int UPDATE_MODE = 0;

// ================================================================
// MODBUS ADDRESS ENUM
// ================================================================
typedef enum {
    PROCESS_VALUE = 0,
    DECIMAL_POINT = 1,
    SET_POINT_1 = 2,
    SET_POINT_2 = 3,
    LOW_ALARM_STATUS = 4,
    HIGH_ALARM_STATUS = 5
} SensorAddress;

// ================================================================
// HELPER FUNCTIONS
// ================================================================

void safeNotify(const String& message) {
    if (deviceConnected && pNotifyCharacteristic != nullptr) {
        pNotifyCharacteristic->setValue(message);
        pNotifyCharacteristic->notify();
    }
}

bool validateInterval(int interval) {
    return (interval >= MIN_UPDATE_INTERVAL && interval <= MAX_UPDATE_INTERVAL);
}

bool validateSetpoint(float value) {
    return (value >= MIN_SETPOINT && value <= MAX_SETPOINT);
}

void setupSD() {
    Serial.print(">> SD: Initializing... ");

    sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (!SD.begin(SD_CS_PIN, sdSPI, 1000000)) { // 4 MHz = stable
        Serial.println("Failed");
        sdReady = false;
        return;
    }

    if (SD.cardType() == CARD_NONE) {
        Serial.println("No card");
        sdReady = false;
        return;
    }

    sdReady = true;
    Serial.println("OK");
}


bool saveDataOffline(const String& timestamp, const String& sensorData) {
    if (!sdReady) {
        Serial.println(">> SD: Not ready");
        return false;
    }

    unsigned long start = millis();

    String filename = "/" + timestamp;
    filename.replace(" ", "");
    filename.replace("-", "");
    filename.replace(":", "");
    filename += ".txt";

    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println(">> SD: Open failed");
        return false;
    }

    file.print(timestamp);
    file.print(",");
    file.print(sensorData);
    file.print("\n");
    file.close();

    if (millis() - start > SD_OPERATION_TIMEOUT) {
        Serial.println(">> SD: Write timeout");
        return false;
    }

    Serial.println(">> SD: Saved -> " + filename);
    return true;
}

void processOfflineFiles() {
    if (!sdReady || WiFi.status() != WL_CONNECTED) return;

    unsigned long start = millis();

    File root = SD.open("/");
    if (!root) return;

    int processed = 0;
    const int MAX_FILES = 5;

    while (processed < MAX_FILES) {
        File file = root.openNextFile();
        if (!file) break;

        if (millis() - start > SD_OPERATION_TIMEOUT) {
            Serial.println(">> SD: Processing timeout");
            file.close();
            break;
        }

        String filename = file.name();
        if (!filename.endsWith(".txt")) {
            file.close();
            continue;
        }

        String content = file.readStringUntil('\n');
        file.close();

        int comma = content.indexOf(',');
        if (comma < 0) {
            SD.remove(filename);
            continue;
        }

        String ts  = content.substring(0, comma);
        String val = content.substring(comma + 1);

        String url = API_URL +
            "?device_code=" + DEVICE_ID +
            "&field1=" + val +
            "&timestamp=" + ts;

        url.replace(" ", "%20");

        WiFiClientSecure client;
        client.setInsecure();

        HTTPClient http;
        http.begin(client, url);
        http.setTimeout(HTTP_TIMEOUT);

        int code = http.GET();
        String resp = http.getString();
        http.end();

        if (code == 200 && resp.indexOf("true") >= 0) {
            SD.remove(filename);
            Serial.println(">> SD: Uploaded & deleted " + filename);
            processed++;
        } else {
            Serial.println(">> SD: Upload failed, retry later");
            break;
        }

        yield();
    }

    root.close();
}


void loadConfig() {
    preferences.begin("app_conf", true);
    if (preferences.isKey("data")) {
        String json = preferences.getString("data", "{}");
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, json);

        if (!error) {
            if(doc.containsKey("name")) DEVICE_NICKNAME = doc["name"].as<String>();
            if (doc.containsKey("id")) DEVICE_ID = doc["id"].as<String>();
            if (doc.containsKey("url")) API_URL = doc["url"].as<String>();
            if (doc.containsKey("ntp")) NTP_SERVER = doc["ntp"].as<String>();

            if (doc.containsKey("int")) {
                int interval = doc["int"].as<int>();
                UPDATE_INTERVAL = validateInterval(interval) ? interval : 60;
            }

            if (doc.containsKey("mode")) {
                UPDATE_MODE = doc["mode"].as<int>();
                UPDATE_MODE = (UPDATE_MODE == 0 || UPDATE_MODE == 1) ? UPDATE_MODE : 0;
            }

            if (doc.containsKey("sp1")) {
                float sp1 = doc["sp1"].as<float>();
                setPoint1 = validateSetpoint(sp1) ? sp1 : 0.0;
            }

            if (doc.containsKey("sp2")) {
                float sp2 = doc["sp2"].as<float>();
                setPoint2 = validateSetpoint(sp2) ? sp2 : 0.0;
            }

            Serial.println(">> CONFIG: Loaded and validated.");
        } else {
            Serial.println(">> CONFIG: JSON parse error");
        }
    }
    preferences.end();
}

void saveConfig() {
    preferences.begin("app_conf", false);
    JsonDocument doc;
    doc["name"] = DEVICE_NICKNAME; 
    doc["id"] = DEVICE_ID;
    doc["url"] = API_URL;
    doc["ntp"] = NTP_SERVER;
    doc["int"] = UPDATE_INTERVAL;
    doc["mode"] = UPDATE_MODE;
    doc["sp1"] = setPoint1;
    doc["sp2"] = setPoint2;

    String output;
    serializeJson(doc, output);
    preferences.putString("data", output);
    preferences.end();
    Serial.println(">> CONFIG: Saved to NVS.");
}

void saveNetworkToMemory(const String& ssid, const String& pass) {
    if (ssid.length() == 0) return;

    preferences.begin("wifi_db", false);
    String currentData = preferences.getString("nets", "[]");
    JsonDocument doc;
    deserializeJson(doc, currentData);
    JsonArray array = doc.as<JsonArray>();

    bool found = false;
    for (JsonObject obj : array) {
        if (obj["s"] == ssid) {
            obj["p"] = pass;
            found = true;
            break;
        }
    }

    if (!found) {
        while (array.size() >= MAX_WIFI_NETWORKS_SAVED) {
            array.remove(0);
        }
        JsonObject newObj = array.add<JsonObject>();
        newObj["s"] = ssid;
        newObj["p"] = pass;
    }

    String output;
    serializeJson(doc, output);
    preferences.putString("nets", output);
    preferences.end();
}

bool tryAutoConnect() {
    // 1. Load saved networks
    preferences.begin("wifi_db", true);
    String savedData = preferences.getString("nets", "[]");
    preferences.end();

    if (savedData == "[]") {
        Serial.println(">> AUTO: No saved networks.");
        return false;
    }

    JsonDocument doc;
    deserializeJson(doc, savedData);
    JsonArray savedNets = doc.as<JsonArray>();

    // 2. Iterate and Try Connecting BLINDLY (NO SCANNING HERE)
    for (JsonObject obj : savedNets) {
        String ssid = obj["s"].as<String>();
        String pass = obj["p"].as<String>();
        
        Serial.printf(">> AUTO: Trying to connect to [%s]...\n", ssid.c_str());
        
        WiFi.disconnect();
        WiFi.begin(ssid.c_str(), pass.c_str());
        
        // Wait up to 4 seconds for connection
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 8) { 
            delay(500);
            Serial.print(".");
            attempts++;
        }
        Serial.println();
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(">> AUTO: Success! Connected.");
            return true;
        } else {
            Serial.println(">> AUTO: Failed. Trying next...");
        }
    }
    
    Serial.println(">> AUTO: Could not connect to any saved network.");
    return false;
}

void setupTime() {
    configTime(19800, 0, NTP_SERVER.c_str()); // IST
    Serial.println(">> TIME: Syncing (IST)...");
}

void setupModbus() {
    Serial1.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
    modbus.begin(1, Serial1);
    
    // --- FIX: LOWER TIMEOUT ---
    // Default is 2000ms. Set to 200ms. 
    // If sensor doesn't reply in 200ms, it probably won't reply at all.
    // modbus.setTimeOut(200); 
    
    Serial.println(">> MODBUS: Initialized (9600 baud, 200ms timeout)");
}

uint8_t writeModbusRegister(uint16_t reg, uint16_t value) {
    uint8_t wResult = modbus.writeSingleRegister(reg, value);
    if (wResult == modbus.ku8MBSuccess) {
        Serial.println(">> MODBUS: Write OK");
        return 1;
    } else {
        Serial.printf(">> MODBUS: Write error: %02X\n", wResult);
        return 0;
    }
}

void sendSensorData() {
    // if (WiFi.status() != WL_CONNECTED) {
    //     Serial.println(">> SKIP: WiFi not connected");
    //     return;
    // }

    // Read sensor via Modbus
    String sensorData = "";
    uint8_t result = modbus.readHoldingRegisters(PROCESS_VALUE, 2);

    if (result == modbus.ku8MBSuccess) {
        uint16_t sensorValue = modbus.getResponseBuffer(0);
        // uint16_t decimalpoint = modbus.getResponseBuffer(1);
        sensorData = String(float(sensorValue)/10.0);
        Serial.println(">> SENSOR: " + sensorData + " (Modbus)");
        // Serial.println(">> DECIMAL: " + String(decimalpoint) + " (Modbus)");
    } else {
        Serial.printf(">> MODBUS: Read error: %02X\n", result);
    }

    // Validate sensor data
    if (sensorData.isEmpty()) {
        Serial.println(">> SKIP: No valid sensor data");

        // UNCOMMENT FOR TESTING WITH DUMMY DATA:
        // sensorData = "99.9";
        // Serial.println(">> TEST MODE: Using dummy data");

        if (sensorData.isEmpty()) {
            return; // Exit if no valid data
        }
    }

    lastWatchdogTime = millis();

    // Get timestamp
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char timeStr[25];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);

    // Build URL
    String fullUrl = API_URL + "?device_code=" + DEVICE_ID + 
                    "&field1=" + sensorData + "&timestamp=" + String(timeStr);
    fullUrl.replace(" ", "%20");

    // Send HTTP request
    WiFiClientSecure client;
    client.setInsecure();
    client.setTimeout(HTTP_TIMEOUT / 1000);

    HTTPClient http;
    http.begin(client, fullUrl);
    http.setTimeout(HTTP_TIMEOUT);

    int httpResponseCode = http.GET();
    String response = http.getString();
    http.end();

    Serial.printf(">> HTTP: Status %d\n", httpResponseCode);
    Serial.println(">> HTTP: Body: " + response);

    // Check success and fallback to SD if failed
    if (httpResponseCode == 200 && response.indexOf("true") >= 0) {
        Serial.println(">> HTTP: Success");
    } else {
        Serial.println(">> HTTP: Failed. Saving to SD...");
        saveDataOffline(String(timeStr), sensorData);
    }
}

void clearSavedWifi() {
    preferences.begin("wifi_db", false);
    if (preferences.clear()) {
        Serial.println(">> NVS: Wi-Fi credentials cleared.");
    } else {
        Serial.println(">> NVS: Failed to clear Wi-Fi.");
    }
    preferences.end();
    
    // Also clear standard ESP32 WiFi NVS (just in case)
    WiFi.disconnect(true, true); 
}

// ================================================================
// BLE CALLBACKS
// ================================================================
class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        deviceConnected = true;
        lastWatchdogTime = millis();
        Serial.println(">> EVENT: Phone Connected");
    }

    void onDisconnect(NimBLEServer* pServer) {
        deviceConnected = false;
        Serial.println(">> EVENT: Phone Disconnected");
        delay(100); 
        NimBLEDevice::startAdvertising();
        Serial.println(">> BLE: Advertising Restarted");
    }
};

class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() == 0) return;

        lastWatchdogTime = millis();
        // Serial.print(">> RAW BLE: ");
        // Serial.println(value.c_str());

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, value);

        if (error) {
            Serial.println(">> BLE: JSON parse error");
            return;
        }

        // Handle actions
        if (doc.containsKey("action")) {
            const char* act = doc["action"];

            if (strcmp(act, "scan") == 0) {
                triggerWifiScan = true;
            }
            else if (strcmp(act, "get_conf") == 0) {
                JsonDocument resp;
                resp["name"] = DEVICE_NICKNAME;
                resp["type"] = SENSOR_TYPE; 
                resp["id"] = DEVICE_ID;
                resp["url"] = API_URL;
                resp["ntp"] = NTP_SERVER;
                resp["int"] = UPDATE_INTERVAL;
                resp["mode"] = UPDATE_MODE;
                resp["sp1"] = setPoint1;
                resp["sp2"] = setPoint2;

                String out;
                serializeJson(resp, out);
                safeNotify(out);
            }
            else if (strcmp(act, "get_status") == 0) {
                String statusMsg = WiFi.status() == WL_CONNECTED 
                    ? "Connected! SSID: " + WiFi.SSID() + " | IP: " + WiFi.localIP().toString()
                    : "Status: Not Connected";
                safeNotify(statusMsg);
            }
            else if (strcmp(act, "forget_wifi") == 0) {
                Serial.println(">> CMD: Forget Wi-Fi requested.");
                
                // 1. Clear Memory
                clearSavedWifi();
                
                // 2. Notify Phone
                if(deviceConnected) {
                    safeNotify("Wi-Fi credentials erased.");
                }
                
                // 3. Disconnect WiFi immediately
                WiFi.disconnect();
            }
            else if (strcmp(act, "ping") == 0) {
                // UNCOMMENT THIS TO SEE IF HEARTBEAT IS ARRIVING
                Serial.print("."); 
            }
        }
        // Handle config updates
        else if (
            doc.containsKey("name") ||
            doc.containsKey("id") || 
            doc.containsKey("url") || 
            doc.containsKey("ntp") || 
            doc.containsKey("int") || 
            doc.containsKey("mode") || 
            doc.containsKey("sp1") || 
            doc.containsKey("sp2") 
        ) {
            bool changed = false;
            bool nameChanged = false;

            if(doc.containsKey("name")) { 
                DEVICE_NICKNAME = doc["name"].as<String>(); 
                changed = true; 
                nameChanged = true;
            }

            if (doc.containsKey("id")) {
                DEVICE_ID = doc["id"].as<String>();
                changed = true;
            }
            if (doc.containsKey("url")) {
                API_URL = doc["url"].as<String>();
                changed = true;
            }
            if (doc.containsKey("ntp")) {
                NTP_SERVER = doc["ntp"].as<String>();
                changed = true;
            }
            if (doc.containsKey("int")) {
                int interval = doc["int"].as<int>();
                if (validateInterval(interval)) {
                    UPDATE_INTERVAL = interval;
                    changed = true;
                } else {
                    safeNotify("Error: Invalid interval (1-86400)");
                }
            }
            if (doc.containsKey("mode")) {
                int mode = doc["mode"].as<int>();
                if (mode == 0 || mode == 1) {
                    UPDATE_MODE = mode;
                    changed = true;
                }
            }
            if (doc.containsKey("sp1")) {
                float sp1 = doc["sp1"].as<float>();
                if (validateSetpoint(sp1)) {
                    setPoint1 = sp1;
                    changed = true;
                    writeModbusRegister(SET_POINT_1, uint16_t(sp1));
                } else {
                    safeNotify("Error: Invalid setpoint 1");
                }
            }
            if (doc.containsKey("sp2")) {
                float sp2 = doc["sp2"].as<float>();
                if (validateSetpoint(sp2)) {
                    setPoint2 = sp2;
                    changed = true;
                    writeModbusRegister(SET_POINT_2, uint16_t(sp2));
                } else {
                    safeNotify("Error: Invalid setpoint 2");
                }
            }

            if (changed) {
                saveConfig();
                forceHttpNow = true;
                // safeNotify("Settings Saved.");
                if(deviceConnected) {
                    safeNotify(nameChanged ? "Name Saved. Restarting..." : "Settings Saved.");
                }
                if (nameChanged) {
                    delay(1000);
                    ESP.restart();
                }
            }
        }
        // Handle WiFi credentials
        else if (doc.containsKey("ssid")) {
            targetSSID = String((const char*)doc["ssid"]);
            targetPass = doc.containsKey("pass") ? String((const char*)doc["pass"]) : "";
            targetSSID.trim();
            targetPass.trim();

            if (targetSSID.length() > 0) {
                wifiConfigReceived = true;
            }
        }
    }
};

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

    setupSD();

    if (sdReady) {
        Serial.println("Card type: " + String(SD.cardType()));
        Serial.println("Card size: " + String(SD.cardSize() / (1024 * 1024)) + " MB");
    }

    delay(500);

    Serial.println("\n--- FIRMWARE STARTED (v1.1) ---");

    // Initialize preferences
    preferences.begin("wifi_db", false);
    if (!preferences.isKey("nets")) {
        preferences.putString("nets", "[]");
    }
    preferences.end();

    loadConfig();
    setupModbus();

    // Generate Default Name (Original Hardware ID)
    uint64_t mac = ESP.getEfuseMac();
    uint32_t lowBytes = (uint32_t)mac;
    defaultName = "ESP_Setup_" + String(lowBytes, HEX);
    defaultName.toUpperCase();

    loadConfig();

    // Decide which name to use
    String broadcastName;
    if (DEVICE_NICKNAME.length() > 0) {
        broadcastName = DEVICE_NICKNAME;
    } else {
        broadcastName = defaultName;
    }
    
    Serial.println(">> Broadcasting Name: " + broadcastName);

    // Initialize BLE
    NimBLEDevice::init(broadcastName.c_str());
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setMTU(250);

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    NimBLECharacteristic *pWriteCharacteristic = pService->createCharacteristic(
        CHAR_UUID_WRITE,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pWriteCharacteristic->setCallbacks(new MyCallbacks());

    pNotifyCharacteristic = pService->createCharacteristic(
        CHAR_UUID_NOTIFY,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);

    NimBLEAdvertisementData scanResp;
    scanResp.setName(broadcastName.c_str());
    pAdvertising->setScanResponseData(scanResp);
    pAdvertising->start();

    lastWatchdogTime = millis();

    // WiFi setup
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (tryAutoConnect()) {
        Serial.println(">> BOOT: Connected to WiFi");
        setupTime();
    } else {
        Serial.println(">> BOOT: No saved networks found");
    }
}

// ================================================================
// MAIN LOOP
// ================================================================
void loop() {
    // 1. Watchdog
    if (deviceConnected && !watchdogPaused && 
        (millis() - lastWatchdogTime > WATCHDOG_TIMEOUT)) {
        Serial.println(">> WATCHDOG: App timeout. Force disconnect.");
        NimBLEDevice::getServer()->disconnect(0);
    }

    // 2. Process offline files (ONLY if connected)
    if (WiFi.status() == WL_CONNECTED && 
        (millis() - lastFileCheckTime > FILE_CHECK_INTERVAL)) {
        lastFileCheckTime = millis();
        watchdogPaused = true;
        processOfflineFiles();
        watchdogPaused = false;
        lastWatchdogTime = millis();
    }

    // ============================================================
    // 3. SENSOR DATA LOGIC (MOVED OUTSIDE WIFI CHECK)
    // ============================================================
    bool shouldTrigger = false;

    // MODE 0: Interval (Timer)
    if (UPDATE_MODE == 0) {
        if (millis() - lastHttpTime > (UPDATE_INTERVAL * 1000UL)) {
            shouldTrigger = true;
            lastHttpTime = millis();
        }
    }
    // MODE 1: Clock Aligned (Cron)
    else if (UPDATE_MODE == 1) {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            int minInterval = UPDATE_INTERVAL / 60;
            if (minInterval < 1) minInterval = 1;

            if (timeinfo.tm_min % minInterval == 0 && 
                timeinfo.tm_min != lastClockMinute) {
                shouldTrigger = true;
                lastClockMinute = timeinfo.tm_min;
            }
        }
    }

    if (forceHttpNow) {
        shouldTrigger = true;
        forceHttpNow = false;
    }

    if (shouldTrigger) {
        // sendSensorData handles the "If No Wifi -> Save to SD" logic internally
        sendSensorData();
    }
    // ============================================================

    // 4. WiFi scan request
    if (triggerWifiScan) {
        triggerWifiScan = false;
        watchdogPaused = true;
        safeNotify("Scanning...");
        WiFi.disconnect();
        // ... (Keep existing scan logic) ...
        int n = WiFi.scanNetworks();
        JsonDocument scanDoc;
        JsonArray array = scanDoc.to<JsonArray>();
        for (int i = 0; i < n && i < MAX_SCAN_RESULTS; ++i) {
            if (WiFi.SSID(i).length() > 0) array.add(WiFi.SSID(i));
        }
        String output;
        serializeJson(scanDoc, output);
        safeNotify(output);
        WiFi.scanDelete();
        watchdogPaused = false;
        lastWatchdogTime = millis();
    }

    // 5. WiFi connection request
    if (wifiConfigReceived) {
        // ... (Keep existing connect logic) ...
        wifiConfigReceived = false;
        watchdogPaused = true;
        safeNotify("Connecting...");
        WiFi.disconnect();
        WiFi.begin(targetSSID.c_str(), targetPass.c_str());
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < WIFI_CONNECT_ATTEMPTS) {
            delay(500); Serial.print("."); attempts++;
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            String msg = "Connected! SSID: " + WiFi.SSID() + " | IP: " + WiFi.localIP().toString();
            Serial.println(">> " + msg);
            safeNotify(msg);
            saveNetworkToMemory(targetSSID, targetPass);
            setupTime();
            forceHttpNow = true;
        } else {
            Serial.println(">> ERROR: WiFi connection failed");
            safeNotify("Connection Failed.");
        }
        watchdogPaused = false;
        lastWatchdogTime = millis();
    }

    // 6. Auto-reconnect (Background)
    if (WiFi.status() != WL_CONNECTED && !deviceConnected && 
        (millis() - lastWifiCheck > WIFI_RECONNECT_INTERVAL)) {
        lastWifiCheck = millis();
        tryAutoConnect();
    }

    delay(10);
}

// #include <Arduino.h>
// #include <nvs_flash.h>
// #include <Preferences.h>

// void setup() {
//     Serial.begin(115200);
//     delay(2000); // Give time to open monitor

//     Serial.println("\n\n-----------------------------------");
//     Serial.println("STARTING FULL MEMORY WIPE...");
//     Serial.println("-----------------------------------");

//     // 1. Erase NVS (Non-Volatile Storage) - Low Level
//     // This clears Wi-Fi creds, Bluetooth bonds, and generic data
//     esp_err_t err = nvs_flash_erase();
//     if (err == ESP_OK) {
//         Serial.println(">> NVS Flash: ERASED SUCCESSFULLY.");
//     } else {
//         Serial.printf(">> NVS Flash: Erase Failed (Error: %s)\n", esp_err_to_name(err));
//     }

//     // 2. Re-Initialize NVS
//     err = nvs_flash_init();
//     if (err == ESP_OK) {
//         Serial.println(">> NVS Flash: RE-INITIALIZED.");
//     }

//     // 3. Clear Preferences (High Level wrapper used in your app)
//     // Specifically targeting the namespaces you used
//     Preferences preferences;
    
//     preferences.begin("app_conf", false);
//     preferences.clear();
//     preferences.end();
//     Serial.println(">> Preferences 'app_conf': CLEARED.");

//     preferences.begin("wifi_db", false);
//     preferences.clear();
//     preferences.end();
//     Serial.println(">> Preferences 'wifi_db': CLEARED.");

//     Serial.println("-----------------------------------");
//     Serial.println("!!! FACTORY RESET COMPLETE !!!");
//     Serial.println("You can now upload your main firmware.");
//     Serial.println("-----------------------------------");
// }

// void loop() {
//     // Do nothing
//     delay(1000);
// }

// ================================================================

