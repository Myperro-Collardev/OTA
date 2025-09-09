#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <Preferences.h>


Preferences prefs; 



// ---------------- Pins & Modem UART ----------------
#define EC200U_RX 5
#define EC200U_TX 6
HardwareSerial ecSerial(1);



// ---------------- State ----------------
bool trackingActive = false;
String output = "";

// ---------------- BLE (NUS) ----------------
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ---------------- Button/LED/RTC ----------------
#define BUTTON_PIN        0
#define LED_PIN           2
#define LONG_PRESS_MS     1000
#define BUTTON_ACTIVE_LVL LOW

RTC_DATA_ATTR uint8_t rtc_state = 0; // 0=OFF(default), 1=ON

// LED config (active-low boards set true)
#define LED_ACTIVE_LOW false

// ---------------- Geofence / GNSS flags ----------------
static bool gpsActive = false;
static bool gpsHasFix = false;



// ---------------- Wi-Fi / Cloud ----------------
String WIFI_SSID = "SSID";
String WIFI_PASS = "PWD";
const String FIREBASE_HOST = "myperro-gps-default-rtdb.firebaseio.com";


// ---------------- LTE sleep control ----------------
static bool lteSleeping = false;

// ---- HTTP OTA config ----
static const char* FW_URL     = "http://YOUR_SERVER/path/to/myperro-esp32c6-v2.bin";
static const char* FW_VERSION = "1.0.0";


class BleSerialTee : public Print {
  public:
    // Forwarders for Serial control and input
    void begin(unsigned long baud) { ::Serial.begin(baud); }
    //void begin(unsigned long baud, uint32_t config) { ::Serial.begin(baud, config); }
    void end() { ::Serial.end(); }
    int available() { return ::Serial.available(); }
    int read() { return ::Serial.read(); }
    int peek() { return ::Serial.peek(); }
    void flush() { ::Serial.flush(); }
    String readString() { return ::Serial.readString(); }
    String readStringUntil(char terminator) { return ::Serial.readStringUntil(terminator); }

    // Write a single byte
    virtual size_t write(uint8_t b) override {
      size_t w = ::Serial.write(b);
      bufferPush(&b, 1);
      return w;
    }

    // Write a buffer
    virtual size_t write(const uint8_t *buffer, size_t size) override {
      size_t w = ::Serial.write(buffer, size);
      bufferPush(buffer, size);
      return w;
    }

    using Print::write; // bring in Print's other write() overloads

  private:
    static const size_t kChunk = 180; // safe MTU payload for NUS
    uint8_t _buf[kChunk];
    size_t  _len = 0;

    void bufferPush(const uint8_t* data, size_t n) {
      while (n > 0) {
        size_t space = kChunk - _len;
        size_t toCopy = (n < space) ? n : space;
        memcpy(_buf + _len, data, toCopy);
        _len += toCopy;
        data += toCopy;
        n -= toCopy;

        // Flush on newline or when buffer is full
        if (_len >= kChunk || (_len && _buf[_len - 1] == '\n')) {
          bleFlush();
        }
      }
    }

    void bleFlush() {
      if (!deviceConnected || pTxCharacteristic == nullptr || _len == 0) {
        _len = 0;
        return;
      }
      size_t sent = 0;
      while (sent < _len) {
        size_t n = (_len - sent > kChunk) ? kChunk : (_len - sent);
        pTxCharacteristic->setValue(_buf + sent, n);
        pTxCharacteristic->notify();
        sent += n;
        delay(1); // yield to BLE stack
      }
      _len = 0;
    }
};

// Create tee instance and re-map Serial to it (after class so ::Serial is resolvable inside)
BleSerialTee SerialTee;
#define Serial SerialTee
// =================== END SERIAL → BLE TEE WRAPPER ===================

// Forward decls
static inline bool buttonIsPressed();
static inline void setLed(bool on);
void connectToWiFi();
int  getWiFiRSSI();
void initializeLTE();
bool getRawLoc(String &lat, String &lon, String &fixTime);
String convertToDecimal(String dms);
void sendToFirebase(const String &payload);
String sendAT(const String &cmd, uint32_t timeout);

// ---- HTTP OTA core ----
static bool httpOtaDownloadAndUpdate(const char* url) {
  Serial.println("[OTA] Starting HTTP OTA…");

  HTTPClient http;
  http.setReuse(false);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.addHeader("User-Agent", "MyPerro-ESP32C6-HTTP-OTA");
  http.addHeader("X-Device-Version", FW_VERSION);
  http.addHeader("X-Device-Chip", String(ESP.getChipModel()));
  http.addHeader("X-Device-FlashSize", String(ESP.getFlashChipSize()));

  WiFiClient client; // for HTTPS use WiFiClientSecure + cert
  if (!http.begin(client, url)) {
    Serial.println("[OTA] http.begin() failed");
    return false;
  }

  int code = http.GET();
  if (code == HTTP_CODE_NOT_MODIFIED) {
    Serial.println("[OTA] Up-to-date (304 Not Modified).");
    http.end();
    return false;
  }
  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] HTTP error: %d\n", code);
    http.end();
    return false;
  }

  int64_t contentLength = http.getSize();
  if (contentLength <= 0) {
    Serial.println("[OTA] Invalid Content-Length");
    http.end();
    return false;
  }

  if (!Update.begin(contentLength)) {
    Serial.printf("[OTA] Update.begin() failed, err=%d\n", Update.getError());
    http.end();
    return false;
  }

  WiFiClient * stream = http.getStreamPtr();
  const size_t kBuf = 4096;
  uint8_t buf[kBuf];
  int64_t written = 0;

  Serial.printf("[OTA] Content-Length: %lld bytes\n", (long long)contentLength);
  while (http.connected() && written < contentLength) {
    size_t avail = stream->available();
    if (avail) {
      size_t toRead = avail > kBuf ? kBuf : avail;
      int rd = stream->readBytes(buf, toRead);
      if (rd <= 0) {
        Serial.println("[OTA] Stream read error");
        Update.abort();
        http.end();
        return false;
      }
      size_t w = Update.write(buf, rd);
      if (w != (size_t)rd) {
        Serial.printf("[OTA] Write mismatch w=%u rd=%d\n", (unsigned)w, rd);
        Update.abort();
        http.end();
        return false;
      }
      written += rd;
    } else {
      delay(1);
    }
  }

  if (!Update.end(true)) {
    Serial.printf("[OTA] Update.end() failed, err=%d\n", Update.getError());
    http.end();
    return false;
  }

  http.end();
  Serial.printf("[OTA] Update successful! Written %lld bytes. Rebooting…\n", (long long)written);
  delay(300);
  ESP.restart();
  return true; // not reached
}

static void checkForHttpOta() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Not connected, skipping OTA.");
    return;
  }
  (void)httpOtaDownloadAndUpdate(FW_URL);
}

// ---- BLE callbacks ----
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer)   { deviceConnected = true; }
  void onDisconnect(BLEServer *pServer){ deviceConnected = false; }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    rxValue.toLowerCase();
    rxValue.trim();

    if (rxValue.length() > 0) {
      Serial.println(rxValue);

      String digits = "";
      for (int i = 0; i < output.length(); i++) {
        if (isdigit(output.charAt(i))) digits += output.charAt(i);
      }

      if (rxValue == "send imei") {
        pTxCharacteristic->setValue(digits.c_str());
        pTxCharacteristic->notify();
        Serial.print("Sent via BLE: ");
        Serial.println(digits);
      }
    }
  }
};

void sendATCommand(String command) {
  Serial.print("Sending command: ");
  Serial.println(command);
  ecSerial.println(command);
  delay(500);
}

// ---- Helpers ----
static inline bool buttonIsPressed() {
  return digitalRead(BUTTON_PIN) == BUTTON_ACTIVE_LVL;
}
static inline void setLed(bool on) {
  int want = on ? HIGH : LOW;
  if (LED_ACTIVE_LOW) want = on ? LOW : HIGH;
  if (digitalRead(LED_PIN) != want) digitalWrite(LED_PIN, want);
}

static inline void setLteSleep(bool enable) {
  if (enable) {
    if (!lteSleeping) {
      ecSerial.println("AT+CFUN=0");
      uint32_t t0 = millis();
      while (millis() - t0 < 300) { while (ecSerial.available()) (void)ecSerial.read(); }
      lteSleeping = true;
      Serial.println("🌙 EC200U → sleep (CFUN=0)");
    }
  } else {
    if (lteSleeping) {
      ecSerial.println("AT+CFUN=1");
      uint32_t t0 = millis();
      while (millis() - t0 < 400) { while (ecSerial.available()) (void)ecSerial.read(); }
      lteSleeping = false;
      Serial.println("☀️ EC200U → awake (CFUN=1)");
    }
  }
}

static bool isGpsErrorToken(const String &s) {
  String t = s; t.toUpperCase();

  int cmeIdx = t.indexOf("+CME ERROR");
  if (cmeIdx >= 0) {
    int colon = t.indexOf(':', cmeIdx);
    if (colon >= 0) {
      int i = colon + 1;
      while (i < (int)t.length() && (t[i] == ' ')) i++;
      int start = i;
      while (i < (int)t.length() && isdigit((unsigned char)t[i])) i++;
      String code = t.substring(start, i);
      if (code == "516") return false;
      return true;
    }
    return true;
  }
  if (t.indexOf("GPS NOT FIX") >= 0) return true;
  if (t.indexOf("NO FIX") >= 0) return true;
  if (t.indexOf("LOCATION UNKNOWN") >= 0) return true;
  if (t.indexOf("ERROR") >= 0) return true;
  return false;
}

static void ec200uSoftResetAndReinitLTE() {
  Serial.println("⚠️ EC200U failsafe: Soft reset via AT+CFUN=1,1");
  ecSerial.flush();
  ecSerial.println("AT+CFUN=1,1");
  uint32_t t0 = millis();
  delay(200);
  while (millis() - t0 < 8000) { while (ecSerial.available()) (void)ecSerial.read(); }

  Serial.println("♻️ Re-initializing LTE context...");
  ecSerial.println("AT");               delay(300);
  ecSerial.println("AT+CFUN=1");        delay(300);
  ecSerial.println("AT+QICSGP=1,1,\"airtelgprs.com\",\"\",\"\",1"); delay(300);
  ecSerial.println("AT+CGATT=1");       delay(300);
  ecSerial.println("AT+QIACT=1");       delay(1200);

  if (trackingActive) {
    Serial.println("🛰️ Restarting GNSS after reset...");
    ecSerial.println("AT+QGPS=1"); delay(500);
    gpsActive = true;
    gpsHasFix = false;
  }
  lteSleeping = false;
}

static bool recoverIfGpsError(const String &resp, const char* where) {
  if (!isGpsErrorToken(resp)) return false;
  Serial.print("❗ Detected GPS/Modem error at "); Serial.println(where);
  ec200uSoftResetAndReinitLTE();
  return true;
}

static void serviceLed(uint32_t /*now*/, bool outside) {
  if (WiFi.status() != WL_CONNECTED || outside) setLed(false);
  else setLed(true);
}

// ---- Deep sleep ----
static void goToSleep() {
  esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  rtc_gpio_pullup_en((gpio_num_t)BUTTON_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)BUTTON_PIN);
  Serial.println("→ Entering deep sleep now...");
  delay(50);
  esp_deep_sleep_start();
  Serial.println("Sleep Check Fail !!");
}

// ---- Setup ----
void setup() {
  pinMode(LED_PIN, OUTPUT);
  setLed(false);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(50);

prefs.begin("Data", false);
String CollarName = prefs.getString("CollarName", "FAILED!");
String WIFI_SSID = prefs.getString("wifi_ssid", "FAILED!");
String WIFI_PASS = prefs.getString("wifi_pwd", "FAILED!");
prefs.end();
WIFI_SSID.trim();
WIFI_PASS.trim();

Serial.println(WIFI_SSID);
Serial.println(WIFI_PASS);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  switch (cause) {
    case ESP_SLEEP_WAKEUP_GPIO:      Serial.println("Wake cause: GPIO"); break;
    case ESP_SLEEP_WAKEUP_TIMER:     Serial.println("Wake cause: TIMER"); break;
    case ESP_SLEEP_WAKEUP_UNDEFINED: Serial.println("Power-on / reset (no deep-sleep wake cause)"); break;
    default:                         Serial.printf("Wake cause: %d\n", (int)cause); break;
  }

  if (rtc_state == 1) {
    Serial.println("RTC state=ON → Staying awake (normal run).");
    ecSerial.begin(115200, SERIAL_8N1, EC200U_RX, EC200U_TX);
    delay(200);
    connectToWiFi();
    checkForHttpOta();   // OTA only if Wi-Fi connected
    return;
  }

  Serial.println("RTC state=OFF → Checking for long press to turn ON...");

  if (!buttonIsPressed()) {
    Serial.println("Button not held → sleeping.");
    goToSleep();  // never returns
  }

  uint32_t t0 = millis();
  while (buttonIsPressed()) {
    if (millis() - t0 >= LONG_PRESS_MS) {
      rtc_state = 1;
      Serial.println("Long press detected → state=ON. Running normally.");
      setLed(true);
      Serial.println(CollarName);

      ecSerial.begin(115200, SERIAL_8N1, EC200U_RX, EC200U_TX);
      delay(500);
      connectToWiFi();
      checkForHttpOta(); // OTA only if Wi-Fi connected

      Serial.println("Setup complete. Sending AT and AT+GSN commands to the EC200U-CN.");
      sendATCommand("AT");
      delay(1000);
      sendATCommand("AT+GSN");

      BLEDevice::init("UART Service");
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());
      BLEService *pService = pServer->createService(SERVICE_UUID);

      pTxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_TX,
                            BLECharacteristic::PROPERTY_NOTIFY
                          );
      pTxCharacteristic->addDescriptor(new BLE2902());

      BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                               CHARACTERISTIC_UUID_RX,
                                               BLECharacteristic::PROPERTY_WRITE
                                             );
      pRxCharacteristic->setCallbacks(new MyCallbacks());

      pService->start();
      pServer->getAdvertising()->start();
      Serial.println("Waiting for a client connection...");

      if (ecSerial.available()) {
        String response = ecSerial.readString();
        Serial.print("EC200U-CN Raw Response: ");
        output = response;
        Serial.println(output);
      }
      return;
    }
    delay(10);
  }

  Serial.println("Short press only → sleeping again.");
  goToSleep();  // never returns
}

// ---- Loop ----
void loop() {
  delay(100);

  if (deviceConnected && Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  int rssi = getWiFiRSSI();
  Serial.printf("📶 RSSI: %d dBm\n", rssi);

  bool inside  = (rssi >= -60);
  bool outside = (rssi < -70 || rssi == 0);

  serviceLed(millis(), outside);

  if (outside) {
    Serial.println("🚪 Outside Wi-Fi geofence");

    if (lteSleeping) setLteSleep(false);

    if (!trackingActive) {
      initializeLTE();
      String __qgps1 = sendAT("AT+QGPS=1", 500);
      recoverIfGpsError(__qgps1, "AT+QGPS=1");
      trackingActive = true;
      gpsActive = true;
      gpsHasFix = false;
    }

    String latRaw, lonRaw, fixTime;
    if (getRawLoc(latRaw, lonRaw, fixTime)) {
      gpsHasFix = true;
      String latDD = convertToDecimal(latRaw);
      String lonDD = convertToDecimal(lonRaw);
      Serial.printf("🕒 Fix Time: %s | 📍 Lat: %s | Lon: %s\n", fixTime.c_str(), latDD.c_str(), lonDD.c_str());
      setLed(true);

      DynamicJsonDocument doc(256);
      doc["timestamp"] = fixTime;
      doc["latitude"]  = latDD;
      doc["longitude"] = lonDD;

      String payload;
      serializeJson(doc, payload);
      sendToFirebase(payload);
    } else {
      Serial.println("❌ No GPS fix.");
      gpsHasFix = false;
    }

    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

  } else if (inside) {
    if (trackingActive) {
      Serial.println("🛑 Back in geofence, disabling tracking");
      sendAT("AT+QGPSEND", 300);
      sendAT("AT+QIDEACT=1", 300);
      trackingActive = false;
      gpsActive = false;
      gpsHasFix = false;
      setLteSleep(true);
    } else {
      Serial.println("📍 Inside geofence, idle");
      if (WiFi.status() == WL_CONNECTED) {
        setLteSleep(true);
      }
    }
  }
}

// ---- Wi-Fi helpers ----
void connectToWiFi() {
  
  prefs.begin("Data", false);
  String WIFI_SSID = prefs.getString("wifi_ssid", "FAILED!");
  String WIFI_PASS = prefs.getString("wifi_pwd", "FAILED!");
  prefs.end();
  Serial.print("🔌 Connecting to Wi-Fi " );
  Serial.print(WIFI_SSID);
  Serial.print(" with Password ");
  Serial.println(WIFI_PASS);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Wi-Fi connected");
    setLteSleep(true);
  } else {
    Serial.println("\n❌ Wi-Fi not found.");
  }
}

int getWiFiRSSI() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.RSSI();
  } else {
    Serial.println("❌ Wi-Fi not connected. RSSI = 0");
    return 0;
  }
}

// ---- LTE + GNSS ----
void initializeLTE() {
  lteSleeping = false;
  sendAT("AT", 200);
  sendAT("AT+CFUN=1", 300);
  sendAT("AT+QICSGP=1,1,\"airtelgprs.com\",\"\",\"\",1", 300);
  sendAT("AT+CGATT=1", 300);
  sendAT("AT+QIACT=1", 1000);
}

bool getRawLoc(String &lat, String &lon, String &fixTime) {
  String res = sendAT("AT+QGPSLOC?", 1000);

  if (recoverIfGpsError(res, "AT+QGPSLOC?")) return false;

  int i = res.indexOf("+QGPSLOC:");
  if (i < 0) return false;
  String line = res.substring(i + 10, res.indexOf("\r", i));
  fixTime = line.substring(0, 6);  // hhmmss
  int p1 = line.indexOf(','), p2 = line.indexOf(',', p1 + 1), p3 = line.indexOf(',', p2 + 1);
  lat = line.substring(p1 + 1, p2);
  lon = line.substring(p2 + 1, p3);
  return true;
}

String convertToDecimal(String dms) {
  char hemi = dms[dms.length() - 1];
  dms.remove(dms.length() - 1);
  int degDigits = (dms.length() == 9) ? 2 : 3; // 2 for lat, 3 for lon
  float deg = dms.substring(0, degDigits).toFloat();
  float min = dms.substring(degDigits).toFloat();
  float dec = deg + (min / 60.0);
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return String(dec, 6);
}

void sendToFirebase(const String &payload) {
  sendAT("AT+QHTTPCFG=\"contextid\",1", 200);
  sendAT("AT+QHTTPCFG=\"requestheader\",0", 200);
  sendAT("AT+QHTTPCFG=\"sslctxid\",1", 200);
  sendAT("AT+QSSLCFG=\"sslversion\",1,4", 200);
  sendAT("AT+QSSLCFG=\"seclevel\",1,0", 200);
  sendAT("AT+QHTTPCFG=\"url\",\"https://" + FIREBASE_HOST + "/locations.json?print=silent\"", 500);

  ecSerial.println("AT+QHTTPPOST=" + String(payload.length()) + ",60,60");
  delay(200);
  if (ecSerial.find("CONNECT")) {
    ecSerial.print(payload);
    Serial.println("📤 Sent: " + payload);
  } else {
    Serial.println("❌ POST failed");
  }

  delay(800);
  while (ecSerial.available()) Serial.write(ecSerial.read());
}

String sendAT(const String &cmd, uint32_t timeout) {
  ecSerial.println(cmd);
  String resp;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (ecSerial.available()) resp += char(ecSerial.read());
  }
  Serial.printf("🔁 %s\n%s\n", cmd.c_str(), resp.c_str());
  return resp;
}

