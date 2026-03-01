#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
// Include fonts required by the UI
// Fonts are included by TFT_eSPI if defined in setup
#include <ArduinoJson.h>
#include <math.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <PNGdec.h>
#include <time.h>
#include <WebServer.h>
#include "FS.h"
#include "SD.h"
#include <LittleFS.h>
#include <PubSubClient.h>

#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

#ifndef MQTT_HOST
#define MQTT_HOST "192.168.0.19"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_USER
#define MQTT_USER ""
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif


// --- CONFIGURATION ---
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const int UDP_PORT = 4210;
const char* camStreamURL = "http://192.168.0.196/stream";      // ESP32-CAM stream endpoint
const char* camSnapshotURL = "http://192.168.0.196/capture"; // Optional snapshot endpoint

const char* mqttHost = MQTT_HOST;
const uint16_t mqttPort = MQTT_PORT;
const char* mqttUser = MQTT_USER;
const char* mqttPassword = MQTT_PASSWORD;
const char* mqttClientIdBase = "4indisplay";
const unsigned long MQTT_RETRY_MS = 5000;
const char* TZ_INFO = "CST6CDT,M3.2.0/2,M11.1.0/2";
const char* MQTT_STATUS_TOPIC = "gate/status";
const char* MQTT_LOG_TOPIC = "gate/log";

const bool RUN_TOUCH_CALIBRATION = false;

// --- PINS (E32R40T) ---
#define RED_LED   22
#define GREEN_LED 16
#define BLUE_LED  17
#define LCD_BL    27
#define SD_CS     5

// Location for weather fetch
const float LATITUDE = 0.0f;
const float LONGITUDE = 0.0f;

// Forward declarations
void fetchWeather();
void drawMainStatus();
void processUDP();
void wakeScreen();
void updateCameraFeed();
void stopCameraStream();
void drawCamBackButton();
void drawCamStatusOverlay();
void showStartupLogo();
void updateStartupStatus(const String &wifiStatus, const String &mqttStatus);
bool connectMqtt(unsigned long timeoutMs);
const char* mqttStateText(int8_t state);
bool attemptMqttConnect();
String sanitizeMqttField(const char* raw);
void handleTouch(uint16_t x, uint16_t y);
void drawMenu();
void drawMenuButton(int pos, String label, uint16_t color);
void logEvent(String event);
String getTimestampString();
void publishStatus(bool retain);
void publishLogEvent(const String &entry);
void handleRoot();
void handleStatusJson();
void handleLog();
void handleEraseLog();
void calibrateTouch();
void testTouchSPI();  // Debug function from touch_debug.cpp

// PNG streaming callbacks
void * myOpen(const char *filename, int32_t *size);
void myClose(void *handle);
int32_t myRead(PNGFILE *handle, uint8_t *buffer, int32_t length);
int32_t mySeek(PNGFILE *handle, int32_t position);

struct CamUrlParts {
  String host;
  String path;
  uint16_t port;
};

bool parseHttpUrl(const char* url, CamUrlParts &parts) {
  if (!url) return false;
  String s(url);
  if (!s.startsWith("http://")) return false;
  s.remove(0, 7);
  int slash = s.indexOf('/');
  String hostPort = (slash >= 0) ? s.substring(0, slash) : s;
  parts.path = (slash >= 0) ? s.substring(slash) : "/";
  int colon = hostPort.indexOf(':');
  if (colon >= 0) {
    parts.host = hostPort.substring(0, colon);
    parts.port = (uint16_t)hostPort.substring(colon + 1).toInt();
  } else {
    parts.host = hostPort;
    parts.port = 80;
  }
  return parts.host.length() > 0;
}

TFT_eSPI tft = TFT_eSPI();
WiFiUDP udp;
PNG png;

WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);
bool mqttConnected = false;
bool timeSynced = false;
unsigned long lastMqttAttempt = 0;

WebServer webServer(80);

WiFiClient camClient;
CamUrlParts camParts;
bool camStreamReady = false;
String camBoundary;
uint8_t *camFrameBuf = nullptr;
size_t camFrameBufSize = 0;
unsigned long camLastFrameTime = 0;
unsigned long camFrameCount = 0;

enum AppState { MAIN_STATUS, MENU_PAGE, WIFI_SETTINGS, LOG_VIEW, ERASE_CONFIRM, CAM_VIEW };
AppState currentUI = MAIN_STATUS;

unsigned long lastActivity = 0;
const unsigned long TIMEOUT_MS = 30000;      
const unsigned long DIM_TIMEOUT = 300000;    
const int BRIGHT_FULL = 255;
const int BRIGHT_DIM = 64; // 25% brightness
bool isDimmed = false;
unsigned long lastGateChangeTime = 0; // Track when gate state last changed

String weatherStr = "Waiting...";
unsigned long lastWeatherTime = 0;
unsigned long lastPacketTime = 0;
int gateState = 2; // 0=Closed, 1=Open, 2=Lost Signal
int lastDrawnGate = -1;

// TJpg_Decoder Callback
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if ( y >= tft.height() ) return false;
  tft.pushImage(x, y, w, h, bitmap);
  return true;
}

// PNGdec Callback
int pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[480];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(0, pDraw->y, pDraw->iWidth, 1, lineBuffer);
  return 1;
}

// PNGdec File callbacks for LittleFS streaming
static File pngFile;

void * myOpen(const char *filename, int32_t *size) {
  pngFile = LittleFS.open(filename, "r");
  *size = pngFile.size();
  return &pngFile;
}

void myClose(void *handle) {
  if (pngFile) pngFile.close();
}

int32_t myRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!pngFile) return 0;
  return pngFile.read(buffer, length);
}

int32_t mySeek(PNGFILE *handle, int32_t position) {
  if (!pngFile) return 0;
  return pngFile.seek(position);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("--- Booting ---");
  
  // Explicitly ensure CS pins are high to prevent bus conflict
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH); // SD CS
  pinMode(TOUCH_CS, OUTPUT); digitalWrite(TOUCH_CS, HIGH); // Touch CS
  pinMode(15, OUTPUT); digitalWrite(15, HIGH); // TFT CS

  Serial.println("Initializing TFT...");
  tft.init();
  tft.setRotation(1);
  showStartupLogo();
  // updateStartupStatus("WiFi: connecting...", "MQTT: waiting...");
  
  Serial.println("Initializing Touch...");
  // Pre-initialize touch pins before TFT_eSPI tries to use them
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  delay(50);
  
  if (!RUN_TOUCH_CALIBRATION) {
    // Replace these with the calibration values printed in Serial.
    uint16_t calData[5] = {376, 3456, 448, 3171, 7};
    tft.setTouch(calData);
  }
  delay(50);
  
  // Ensure Backlight is ON
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  
  // Turn off RGB LEDs (Active Low, so HIGH is off)
  pinMode(RED_LED, OUTPUT); digitalWrite(RED_LED, HIGH);
  pinMode(GREEN_LED, OUTPUT); digitalWrite(GREEN_LED, HIGH);
  pinMode(BLUE_LED, OUTPUT); digitalWrite(BLUE_LED, HIGH);

  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);
  
  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized.");
  }
  
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  // Non-blocking WiFi check in loop, or simple wait here
  // For now simple wait with timeout to not hang boot if wifi down
  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifi < 10000) { 
    delay(500); 
    Serial.print("."); 
  }
  Serial.println("\nWiFi Init Complete.");
    if (WiFi.status() == WL_CONNECTED) {
      configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov", "time.google.com");
      struct tm timeinfo;
      timeSynced = getLocalTime(&timeinfo, 5000);
      if (!timeSynced) {
        Serial.println("[WARN] NTP sync failed, using millis timestamps");
      }

      webServer.on("/", handleRoot);
      webServer.on("/status", handleStatusJson);
      webServer.on("/log", handleLog);
      webServer.on("/erase", HTTP_POST, handleEraseLog);
      webServer.begin();
      Serial.print("Web UI available at http://");
      Serial.println(WiFi.localIP());
    }

  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setSocketTimeout(5);
  mqttClient.setKeepAlive(30);
  mqttConnected = connectMqtt(8000);
  // updateStartupStatus("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "connected" : "failed"),
  //                     String("MQTT: ") + (mqttConnected ? "connected" : "failed"));
  
  delay(10000);  // Show startup logo for 10 seconds
  
  udp.begin(UDP_PORT);
  fetchWeather();
  lastActivity = millis();
  
  // Debug touch SPI communication
  // testTouchSPI();
  
  // Run touch calibration on startup
  if (RUN_TOUCH_CALIBRATION) {
    calibrateTouch();
  }
  
  drawMainStatus();
}

void loop() {
  processUDP();

  if (WiFi.status() == WL_CONNECTED) {
    webServer.handleClient();
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      mqttConnected = false;
      if (millis() - lastMqttAttempt > MQTT_RETRY_MS) {
        lastMqttAttempt = millis();
        mqttConnected = attemptMqttConnect();
        if (mqttConnected) {
          publishStatus(true);
        }
      }
    } else {
      mqttClient.loop();
    }
  }

  if (currentUI != MAIN_STATUS && (millis() - lastActivity > TIMEOUT_MS)) {
    currentUI = MAIN_STATUS;
    wakeScreen();
    drawMainStatus();
  }

  // Refresh Camera if viewing
  if (currentUI == CAM_VIEW) {
    updateCameraFeed(); 
  } else {
    stopCameraStream();
  }

  // Poll for Weather occasionally
  if (currentUI == MAIN_STATUS && millis() - lastWeatherTime > 300000) { // 5 min
      fetchWeather();
      drawMainStatus(); // Update screen
  }

  // Redraw main status only if state changed
  if (currentUI == MAIN_STATUS && gateState != lastDrawnGate) {
    drawMainStatus();
    lastDrawnGate = gateState;
  }

  // Dim screen if gate is closed for 5 minutes
  if (currentUI == MAIN_STATUS && gateState == 0) { // Gate Closed
    if (!isDimmed && (millis() - lastGateChangeTime > DIM_TIMEOUT)) {
      analogWrite(LCD_BL, BRIGHT_DIM);
      isDimmed = true;
    }
  } else {
    // If not on main status or gate not closed, ensure screen is bright
    if (isDimmed) {
      wakeScreen();
    }
  }

  // Touch Handling
#ifdef TOUCH_CS
  static unsigned long lastTouchTime = 0;
  uint16_t x, y;
  
  // getTouch returns raw coordinates, calibrated by setTouch()
  if (tft.getTouch(&x, &y, 50)) {
    if (millis() - lastTouchTime > 200) {  // Debounce
      wakeScreen();
      handleTouch(x, y);
      lastTouchTime = millis();
    }
  }
#endif
}

void showStartupLogo() {
  Serial.println("Loading startup logo...");
  tft.fillScreen(TFT_BLACK);
  if (!LittleFS.begin()) {
    Serial.println("[ERROR] LittleFS mount failed");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("LittleFS mount failed", tft.width() / 2, tft.height() / 2);
    return;
  }
  Serial.println("LittleFS mounted successfully");

  // Load and display JPG logo centered
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  
  // Get image dimensions to center it
  uint16_t w = 0, h = 0;
  TJpgDec.getFsJpgSize(&w, &h, "/logo.jpg", LittleFS);
  
  // Calculate centered position
  int16_t x = (tft.width() - w) / 2;
  int16_t y = (tft.height() - h) / 2;
  
  Serial.printf("Logo size: %dx%d, centered at (%d,%d)\n", w, h, x, y);
  
  int result = TJpgDec.drawFsJpg(x, y, "/logo.jpg", LittleFS);
  
  if (result == 0) {
    Serial.println("Logo displayed successfully");
  } else {
    Serial.printf("[ERROR] JPG decode failed with code: %d\n", result);
    // Fallback to text logo
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setFreeFont(&FreeSansBold24pt7b);
    tft.drawString("MHogue Tech", tft.width() / 2, tft.height() / 2 - 30);
    
    tft.setTextFont(2);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("ESP32", tft.width() / 2, tft.height() / 2 + 20);
    tft.drawString("Gate Monitor", tft.width() / 2, tft.height() / 2 + 45);
  }
}

void updateStartupStatus(const String &wifiStatus, const String &mqttStatus) {
  int statusH = 70;
  int y = tft.height() - statusH;
  tft.fillRect(0, y, tft.width(), statusH, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2);
  tft.drawString(wifiStatus, 10, y + 8);
  tft.drawString(mqttStatus, 10, y + 34);
}

bool connectMqtt(unsigned long timeoutMs) {
  if (WiFi.status() != WL_CONNECTED) return false;
  unsigned long start = millis();
  while (!mqttClient.connected() && millis() - start < timeoutMs) {
    mqttConnected = attemptMqttConnect();
    if (mqttConnected) return true;
    delay(300);
  }
  return mqttClient.connected();
}

String sanitizeMqttField(const char* raw) {
  String value = raw ? String(raw) : String("");
  value.trim();
  while (value.length() >= 2) {
    char first = value.charAt(0);
    char last = value.charAt(value.length() - 1);
    bool doubleQuoted = (first == '"' && last == '"');
    bool singleQuoted = (first == '\'' && last == '\'');
    if (!(doubleQuoted || singleQuoted)) break;
    value = value.substring(1, value.length() - 1);
    value.trim();
  }
  return value;
}

const char* mqttStateText(int8_t state) {
  switch (state) {
    case MQTT_CONNECTION_TIMEOUT: return "Connection timeout";
    case MQTT_CONNECTION_LOST: return "Connection lost";
    case MQTT_CONNECT_FAILED: return "Connect failed";
    case MQTT_DISCONNECTED: return "Disconnected";
    case MQTT_CONNECTED: return "Connected";
    case MQTT_CONNECT_BAD_PROTOCOL: return "Bad protocol";
    case MQTT_CONNECT_BAD_CLIENT_ID: return "Bad client ID";
    case MQTT_CONNECT_UNAVAILABLE: return "Broker unavailable";
    case MQTT_CONNECT_BAD_CREDENTIALS: return "Bad credentials";
    case MQTT_CONNECT_UNAUTHORIZED: return "Unauthorized";
    default: return "Unknown state";
  }
}

bool attemptMqttConnect() {
  String clientId = String(mqttClientIdBase) + "-" + String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFF), HEX);
  String mqttUserSanitized = sanitizeMqttField(mqttUser);
  String mqttPasswordSanitized = sanitizeMqttField(mqttPassword);
  bool useAuth = mqttUserSanitized.length() > 0;
  bool ok = useAuth
    ? mqttClient.connect(clientId.c_str(), mqttUserSanitized.c_str(), mqttPasswordSanitized.c_str())
    : mqttClient.connect(clientId.c_str());

  if (!ok) {
    int8_t state = mqttClient.state();
    Serial.printf("[MQTT] Connect failed: state=%d (%s) host=%s port=%u auth=%s\n",
                  state,
                  mqttStateText(state),
                  mqttHost,
                  mqttPort,
                  useAuth ? "on" : "off");
  } else {
    Serial.printf("[MQTT] Connected: clientId=%s\n", clientId.c_str());
  }

  return ok;
}

void wakeScreen() {
  lastActivity = millis();
  analogWrite(LCD_BL, BRIGHT_FULL);
  isDimmed = false;
}

void processUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buf[255];
    int len = udp.read(buf, 255);
    if (len > 0) buf[len] = 0;
    String msg = String(buf);
    
    // Simple state logic
    int newState = gateState;
    if (msg == "OPEN") newState = 1;
    else if (msg == "CLOSED") newState = 0;
    
    if (newState != gateState) {
        gateState = newState;
        lastGateChangeTime = millis(); // Reset timer on state change
        String event = (gateState == 1) ? "OPEN" : (gateState == 0) ? "CLOSED" : "LOST_SIGNAL";
        logEvent(event);
      publishStatus(true);
        wakeScreen();
    }
    lastPacketTime = millis();
  }
  
  // Timeout for "No Signal"
  if (millis() - lastPacketTime > 15000 && gateState != 2) {
    gateState = 2; // Lost signal
    lastGateChangeTime = millis(); // Reset timer on state change
    logEvent("LOST_SIGNAL");
    publishStatus(true);
    wakeScreen(); // Wake screen to show error state? Or let it sleep?
    // Usually better to show change
  }
}

// Draw the main dashboard UI
void drawMainStatus() {
  // Clear screen with state color
  uint16_t bgColor = (gateState == 1) ? TFT_RED : (gateState == 0 ? TFT_GREEN : TFT_ORANGE);
  tft.fillScreen(bgColor);

  // Set text color for contrast
  uint16_t textColor = (gateState == 1) ? TFT_WHITE : TFT_BLACK; // Red needs White text, others Black
  tft.setTextColor(textColor, bgColor);
  
  // Main Status Text
  tft.setFreeFont(&FreeSansBold24pt7b); // Use nice large font
  
  int cx = tft.width() / 2;
  int cy = tft.height() / 2;
  
  String stateText = (gateState == 1) ? "GATE OPEN!" : (gateState == 0) ? "GATE CLOSED" : "NO SIGNAL";
  tft.setTextDatum(MC_DATUM); // Middle Center
  tft.setTextSize(1);
  
  // Center all status text
  tft.drawString(stateText, cx, cy);

  // Weather / Footer Text - adjust position since main text is now centered
  if (gateState == 0 && weatherStr.length() > 0) {
      tft.setFreeFont(&FreeSansBold12pt7b);
      tft.drawString(weatherStr, cx, cy + 70);
      tft.setTextFont(1); // Revert to default
      tft.setTextSize(2);
      tft.drawString("Liberty, MO", cx, cy + 100);
  }

  // IP address at bottom
  String ipText = (WiFi.status() == WL_CONNECTED) ? (String("IP: ") + WiFi.localIP().toString()) : String("IP: --");
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(textColor, bgColor);
  tft.drawString(ipText, cx, tft.height() - 12);

  // Connection status icons (WiFi + MQTT)
  uint16_t wifiColor = (WiFi.status() == WL_CONNECTED) ? TFT_GREEN : tft.color565(80, 80, 80);
  uint16_t mqttColor = mqttConnected ? TFT_CYAN : tft.color565(80, 80, 80);
  int wifiX = 10;
  int wifiBaseY = 32;
  int barW = 4;
  int barGap = 2;
  int barHeights[3] = {6, 10, 14};
  for (int i = 0; i < 3; i++) {
    int x = wifiX + i * (barW + barGap);
    int h = barHeights[i];
    tft.fillRect(x, wifiBaseY - h, barW, h, wifiColor);
  }
  tft.fillCircle(wifiX + 18, wifiBaseY, 2, wifiColor);

  int mqttX = wifiX + 28;
  int mqttY = 10;
  int mqttW = 44;
  int mqttH = 20;
  tft.fillRoundRect(mqttX, mqttY, mqttW, mqttH, 4, tft.color565(40, 40, 40));
  tft.drawRoundRect(mqttX, mqttY, mqttW, mqttH, 4, mqttColor);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(mqttColor, tft.color565(40, 40, 40));
  tft.drawString("MQTT", mqttX + mqttW / 2, mqttY + mqttH / 2);
  tft.setTextColor(textColor, bgColor);

  // Draw Header Buttons (Menu, Cam)
  // Icons are hard, so we use text or simple shapes
  
  // Camera Button (Left side of header area or next to menu?)
  // Let's put them top right like the React design
  int btnSize = 50;
  int btnY = 10;
  int menuX = tft.width() - btnSize - 10;
  int camX = menuX - btnSize - 10;
  
  // Menu Button
  tft.fillRoundRect(menuX, btnY, btnSize, btnSize, 8, tft.color565(50, 50, 50));
  tft.drawRoundRect(menuX, btnY, btnSize, btnSize, 8, TFT_WHITE);
  // Hamburger icon lines
  tft.fillRect(menuX + 10, btnY + 14, 30, 4, TFT_WHITE);
  tft.fillRect(menuX + 10, btnY + 23, 30, 4, TFT_WHITE);
  tft.fillRect(menuX + 10, btnY + 32, 30, 4, TFT_WHITE);

  // Camera Button
  tft.fillRoundRect(camX, btnY, btnSize, btnSize, 8, tft.color565(50, 50, 50));
  tft.drawRoundRect(camX, btnY, btnSize, btnSize, 8, TFT_WHITE);
  // Camera body + lens + top bump
  tft.drawRoundRect(camX + 10, btnY + 18, 30, 18, 4, TFT_WHITE);
  tft.drawCircle(camX + 25, btnY + 27, 6, TFT_WHITE);
  tft.fillCircle(camX + 25, btnY + 27, 2, TFT_WHITE);
  tft.fillRect(camX + 16, btnY + 12, 14, 6, TFT_WHITE);
}

void updateCameraFeed() {
  if (millis() - camLastFrameTime < 60) return; 

  if (!camStreamReady || !camClient.connected()) {
    camClient.stop();
    camStreamReady = parseHttpUrl(camStreamURL, camParts);
    if (!camStreamReady) return;

    if (!camClient.connect(camParts.host.c_str(), camParts.port)) {
      camStreamReady = false;
      return;
    }

    camClient.setTimeout(2000);
    camClient.print(String("GET ") + camParts.path + " HTTP/1.1\r\n" +
                    "Host: " + camParts.host + "\r\n" +
                    "User-Agent: ESP32\r\n" +
                    "Connection: keep-alive\r\n\r\n");

    camBoundary = "--";
    while (camClient.connected()) {
      String line = camClient.readStringUntil('\n');
      if (line == "\r" || line.length() == 0) break;
      line.trim();
      if (line.startsWith("Content-Type:")) {
        int idx = line.indexOf("boundary=");
        if (idx >= 0) {
          String token = line.substring(idx + 9);
          token.trim();
          if (!token.startsWith("--")) token = "--" + token;
          camBoundary = token;
        }
      }
    }
  }

  String line;
  bool foundBoundary = false;
  while (camClient.connected()) {
    line = camClient.readStringUntil('\n');
    if (line.length() == 0) return;
    line.trim();
    if (line.startsWith(camBoundary)) {
      foundBoundary = true;
      break;
    }
  }
  if (!foundBoundary) return;

  size_t contentLength = 0;
  while (camClient.connected()) {
    line = camClient.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
    line.trim();
    if (line.startsWith("Content-Length:")) {
      contentLength = (size_t)line.substring(15).toInt();
    }
  }
  if (contentLength == 0) return;

  if (contentLength > camFrameBufSize) {
    uint8_t *newBuf = (uint8_t *)realloc(camFrameBuf, contentLength);
    if (!newBuf) return;
    camFrameBuf = newBuf;
    camFrameBufSize = contentLength;
  }

  size_t readTotal = 0;
  while (readTotal < contentLength && camClient.connected()) {
    int readNow = camClient.readBytes(camFrameBuf + readTotal, contentLength - readTotal);
    if (readNow <= 0) break;
    readTotal += (size_t)readNow;
  }
  if (readTotal != contentLength) return;

  tft.fillScreen(TFT_BLACK);
  
  // Get JPEG dimensions and center it
  uint16_t jpgW = 0, jpgH = 0;
  TJpgDec.getJpgSize(&jpgW, &jpgH, camFrameBuf, contentLength);
  
  int16_t x = (tft.width() - jpgW) / 2;
  int16_t y = (tft.height() - jpgH) / 2;
  
  // Ensure we don't go negative
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  
  TJpgDec.drawJpg(x, y, camFrameBuf, contentLength);
  camFrameCount++;
  drawCamBackButton();
  drawCamStatusOverlay();
  camLastFrameTime = millis();
}

void stopCameraStream() {
  camClient.stop();
  camStreamReady = false;
  camFrameCount = 0;
}

void drawCamBackButton() {
  int btnX = 10;
  int btnY = 10;
  int btnW = 80;
  int btnH = 40;
  
  // Draw with bright background for visibility
  tft.fillRoundRect(btnX, btnY, btnW, btnH, 8, TFT_RED);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 8, TFT_WHITE);
  
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.drawString("< BACK", btnX + btnW / 2, btnY + btnH / 2);
}

void drawCamStatusOverlay() {
  // Draw semi-transparent status bar at bottom
  int statusH = 50;
  int statusY = tft.height() - statusH;
  
  // Dark background for readability
  tft.fillRect(0, statusY, tft.width(), statusH, tft.color565(0, 0, 0));
  
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  
  // Show Camera IP Address (not display IP)
  String camIpStr = "Camera: " + camParts.host;
  tft.drawString(camIpStr, 5, statusY + 5);
  
  // Show Display's WiFi Signal (affects stream quality)
  int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
  String signalStr = "RX Signal: ";
  if (rssi >= -50) signalStr += "Excellent";
  else if (rssi >= -60) signalStr += "Good";
  else if (rssi >= -70) signalStr += "Fair";
  else if (rssi >= -80) signalStr += "Weak";
  else signalStr += "Poor";
  signalStr += " (" + String(rssi) + " dBm)";
  tft.drawString(signalStr, 5, statusY + 20);
  
  // Right side: Frame counter
  String frameStr = "Frames: " + String(camFrameCount);
  tft.setTextDatum(TR_DATUM);
  tft.drawString(frameStr, tft.width() - 5, statusY + 5);
  
  // FPS calculation
  static unsigned long lastFpsCalc = 0;
  static unsigned long lastFrameCount = 0;
  static float fps = 0.0;
  
  if (millis() - lastFpsCalc >= 1000) {
    fps = (camFrameCount - lastFrameCount) * 1000.0 / (millis() - lastFpsCalc);
    lastFrameCount = camFrameCount;
    lastFpsCalc = millis();
  }
  
  String fpsStr = "FPS: " + String(fps, 1);
  tft.drawString(fpsStr, tft.width() - 5, statusY + 20);
}

void handleTouch(uint16_t x, uint16_t y) {
  // Button Regions
  // Menu: Top Right
  int btnSize = 50;
  int btnY = 10;
  int menuX = tft.width() - btnSize - 10;
  int camX = menuX - btnSize - 10;

  if (currentUI == MAIN_STATUS) {
    // Check Menu (approx box with padding)
    if (x >= menuX && x <= menuX + btnSize && y >= btnY && y <= btnY + btnSize) {
        currentUI = MENU_PAGE; 
        drawMenu();
        return;
    }
    
    // Check Camera
    if (x >= camX && x <= camX + btnSize && y >= btnY && y <= btnY + btnSize) {
        currentUI = CAM_VIEW;
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_YELLOW);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Connecting Camera...", tft.width()/2, tft.height()/2);
        drawCamBackButton();
        return;
    }
    
    // Debug Sim Buttons (Bottom Left/Right for testing if needed)
    // Left side -> Sim Open
    if (y > tft.height() - 60 && x < 100) {
        gateState = 1; drawMainStatus();
    }
    // Right side -> Sim Closed
    if (y > tft.height() - 60 && x > tft.width() - 100) {
        gateState = 0; drawMainStatus();
    }
    
  } else if (currentUI == CAM_VIEW) {
      // Back button with expanded touch area for better responsiveness
      int btnX = 10;
      int btnY = 10;
      int btnW = 80;
      int btnH = 40;
      
      // Add padding to touch area to make it easier to hit
      int touchPadding = 10;
      
      if (x >= (btnX - touchPadding) && x <= (btnX + btnW + touchPadding) && 
          y >= (btnY - touchPadding) && y <= (btnY + btnH + touchPadding)) {
        currentUI = MAIN_STATUS;
        stopCameraStream();
        drawMainStatus();
        return;
      }
  } else if (currentUI == ERASE_CONFIRM) {
      // Check Confirm button (left)
      if (y >= tft.height()/2 && y <= tft.height()/2 + 50) {
        if (x >= 20 && x <= 150) {
          // Confirmed - delete log
          SD.remove("/gate.log");
          tft.fillScreen(TFT_BLACK);
          tft.setTextColor(TFT_GREEN);
          tft.setTextDatum(MC_DATUM);
          tft.drawString("Log Cleared", tft.width()/2, tft.height()/2);
          delay(2000);
          currentUI = MENU_PAGE;
          drawMenu();
        } else if (x >= 170 && x <= 300) {
          // Cancelled
          currentUI = MENU_PAGE;
          drawMenu();
        }
      }
  } else if (currentUI == MENU_PAGE) {
      // Handle Menu Items
      int menuStartY = 70;
      int itemHeight = 55;
      int itemGap = 10;
      
      // Item 1: View Activity Log
      if (y > menuStartY && y < menuStartY + itemHeight) {
          // View Log
          tft.fillScreen(TFT_BLACK);
          tft.setTextColor(TFT_WHITE);
          tft.setTextDatum(TL_DATUM);
          tft.setFreeFont(&FreeSans9pt7b);
          File file = SD.open("/gate.log");
          if (file) {
            int lineY = 10;
            while (file.available() && lineY < tft.height() - 50) {
              String line = file.readStringUntil('\n');
              tft.drawString(line, 10, lineY);
              lineY += 20;
            }
            file.close();
          } else {
            tft.drawString("No log file found", 10, 50);
          }
          tft.drawString("Tap to return", 10, tft.height() - 30);
          // Wait for touch
          uint16_t dummyX, dummyY;
          while (!tft.getTouch(&dummyX, &dummyY, 100)) delay(100);
          drawMenu();
      }
      // Item 2: Clear Activity Log
        if (y > menuStartY + itemHeight + itemGap &&
          y < menuStartY + itemHeight * 2 + itemGap) {
          // Show confirmation dialog
          currentUI = ERASE_CONFIRM;
          tft.fillScreen(tft.color565(40,40,40));
          tft.setTextColor(TFT_WHITE);
          tft.setTextDatum(MC_DATUM);
          tft.setFreeFont(&FreeSansBold12pt7b);
          tft.drawString("Clear Activity Log?", tft.width()/2, tft.height()/2 - 60);
          
          // Confirm button (Green)
          tft.fillRoundRect(20, tft.height()/2, 130, 50, 8, TFT_GREEN);
          tft.setTextColor(TFT_BLACK, TFT_GREEN);
          tft.drawString("CONFIRM", 85, tft.height()/2 + 25);
          
          // Cancel button (Red)
          tft.fillRoundRect(170, tft.height()/2, 130, 50, 8, TFT_RED);
          tft.setTextColor(TFT_WHITE, TFT_RED);
          tft.drawString("CANCEL", 235, tft.height()/2 + 25);
      }
      // Item 3: Camera View
        if (y > menuStartY + (itemHeight + itemGap) * 2 &&
          y < menuStartY + (itemHeight + itemGap) * 2 + itemHeight) {
          currentUI = CAM_VIEW;
          tft.fillScreen(TFT_BLACK);
          tft.setTextColor(TFT_YELLOW);
          tft.setTextDatum(MC_DATUM);
          tft.drawString("Connecting Camera...", tft.width()/2, tft.height()/2);
          drawCamBackButton();
      }
      // Exit
      if (y > tft.height() - 60) {
          currentUI = MAIN_STATUS;
          drawMainStatus();
      }
  }
}

void drawMenu() {
  tft.fillScreen(tft.color565(20,20,20));
  tft.setTextColor(TFT_WHITE, tft.color565(20,20,20));
  tft.setFreeFont(&FreeSansBold12pt7b);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("SYSTEM MENU", 20, 20);
  
  drawMenuButton(1, "View Activity Log", TFT_BLUE);
  drawMenuButton(2, "Clear Activity Log", TFT_RED);
  drawMenuButton(3, "Camera View", TFT_GREEN);
  
  // Exit Button
  tft.drawRect(20, tft.height() - 60, tft.width() - 40, 50, TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("EXIT", tft.width()/2, tft.height() - 35);
}

void drawMenuButton(int pos, String label, uint16_t color) {
  int y = 70 + (pos-1) * 65;
  tft.fillRoundRect(20, y, tft.width() - 40, 55, 8, tft.color565(40,40,40));
  tft.drawRoundRect(20, y, tft.width() - 40, 55, 8, TFT_WHITE);
  
  // Icon Box
  tft.fillRoundRect(30, y+8, 40, 40, 6, color);
  
  tft.setTextColor(TFT_WHITE, tft.color565(40,40,40));
  tft.setTextDatum(TL_DATUM); // Top Left
  tft.drawString(label, 85, y + 18);
}

void fetchWeather() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    // Replace with real lat/long for Liberty, MO
    // Lat: 39.246, Long: -94.419
    String url = "https://api.open-meteo.com/v1/forecast?latitude=39.246&longitude=-94.419&current_weather=true&temperature_unit=fahrenheit";
    http.begin(url);
    if (http.GET() > 0) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, http.getString());
      if (!err) {
        float temp = doc["current_weather"]["temperature"] | NAN;
        int code = doc["current_weather"]["weathercode"] | -1;
        
        String condition = "Clear";
        if (code > 3) condition = "Cloudy";
        if (code > 50) condition = "Rain";
        if (code > 70) condition = "Snow";
        
        if (!isnan(temp)) {
          weatherStr = String(temp, 1) + "F " + condition;
        }
      }
    }
    http.end();
    lastWeatherTime = millis();
  }
}

String getTimestampString() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 0)) {
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buf);
  }
  return String(millis());
}

void publishStatus(bool retain) {
  if (!mqttClient.connected()) return;
  String gateText = (gateState == 1) ? "OPEN" : (gateState == 0) ? "CLOSED" : "NO_SIGNAL";
  String wifiText = (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
  String mqttText = mqttConnected ? "connected" : "disconnected";
  String timeText = getTimestampString();
  String ipText = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
  long uptimeSec = millis() / 1000;
  int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
  String payload = "{";
  payload += "\"gate\":\"" + gateText + "\",";
  payload += "\"gateState\":" + String(gateState) + ",";
  payload += "\"wifi\":\"" + wifiText + "\",";
  payload += "\"mqtt\":\"" + mqttText + "\",";
  payload += "\"ip\":\"" + ipText + "\",";
  payload += "\"uptime\":" + String(uptimeSec) + ",";
  payload += "\"rssi\":" + String(rssi) + ",";
  payload += "\"weather\":\"" + weatherStr + "\",";
  payload += "\"time\":\"" + timeText + "\"";
  payload += "}";
  mqttClient.publish(MQTT_STATUS_TOPIC, payload.c_str(), retain);
}

void publishLogEvent(const String &entry) {
  if (!mqttClient.connected()) return;
  mqttClient.publish(MQTT_LOG_TOPIC, entry.c_str(), false);
}

void logEvent(String event) {
  if (!SD.exists("/gate.log")) {
    File initFile = SD.open("/gate.log", FILE_WRITE);
    if (initFile) {
      initFile.close();
    }
  }

  File file = SD.open("/gate.log", FILE_APPEND);
  if (file) {
    String ts = getTimestampString();
    String entry = ts + ": " + event;
    file.println(entry);
    file.close();
    publishLogEvent(entry);
  } else {
    Serial.println("Failed to open log file");
  }
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Gate Monitor</title>
  <style>
    :root {
      --bg: #0f1420;
      --panel: #1b2233;
      --accent: #22c55e;
      --warn: #f97316;
      --bad: #ef4444;
      --text: #e5e7eb;
      --muted: #94a3b8;
    }
    * { box-sizing: border-box; font-family: "Trebuchet MS", Verdana, sans-serif; }
    body { margin: 0; background: var(--bg); color: var(--text); }
    header { padding: 18px 22px; background: linear-gradient(120deg, #1f2937, #111827); }
    header h1 { margin: 0; font-size: 22px; letter-spacing: 0.5px; }
    main { padding: 20px; display: grid; grid-template-columns: 1fr; gap: 16px; max-width: 900px; margin: 0 auto; }
    .card { background: var(--panel); border: 1px solid #2b3246; border-radius: 12px; padding: 16px; }
    .row { display: flex; gap: 12px; flex-wrap: wrap; }
    .badge { padding: 6px 10px; border-radius: 999px; font-size: 12px; background: #111827; border: 1px solid #2b3246; }
    .status { font-size: 32px; font-weight: 700; letter-spacing: 0.6px; }
    .status.ok { color: var(--accent); }
    .status.warn { color: var(--warn); }
    .status.bad { color: var(--bad); }
    button { background: #0b1220; color: var(--text); border: 1px solid #2b3246; padding: 10px 14px; border-radius: 10px; cursor: pointer; }
    button:hover { border-color: var(--accent); }
    pre { white-space: pre-wrap; background: #0b1220; padding: 12px; border-radius: 10px; border: 1px solid #2b3246; color: var(--text); max-height: 320px; overflow: auto; }
    .muted { color: var(--muted); font-size: 12px; }
  </style>
</head>
<body>
  <header>
    <h1>MHogue Tech - Gate Monitor</h1>
  </header>
  <main>
    <section class="card">
      <div class="row">
        <span class="badge" id="wifi">WiFi: --</span>
        <span class="badge" id="mqtt">MQTT: --</span>
        <span class="badge" id="time">Time: --</span>
        <span class="badge" id="ip">IP: --</span>
        <span class="badge" id="uptime">Uptime: --</span>
        <span class="badge" id="rssi">RSSI: --</span>
      </div>
      <div style="height: 12px"></div>
      <div class="status" id="gate">--</div>
      <div class="muted" id="weather">--</div>
    </section>

    <section class="card">
      <div class="row" style="justify-content: space-between; align-items: center;">
        <strong>Activity Log</strong>
        <div class="row">
          <button id="refresh">Refresh</button>
          <button id="erase">Erase Log</button>
        </div>
      </div>
      <div style="height: 10px"></div>
      <pre id="log">Loading...</pre>
    </section>
  </main>

  <script>
    async function loadStatus() {
      const res = await fetch('/status');
      const data = await res.json();
      document.getElementById('wifi').textContent = `WiFi: ${data.wifi}`;
      document.getElementById('mqtt').textContent = `MQTT: ${data.mqtt}`;
      document.getElementById('time').textContent = `Time: ${data.time}`;
      document.getElementById('ip').textContent = `IP: ${data.ip}`;
      document.getElementById('uptime').textContent = `Uptime: ${data.uptime}s`;
      document.getElementById('rssi').textContent = `RSSI: ${data.rssi} dBm`;
      const gate = document.getElementById('gate');
      gate.textContent = data.gate;
      gate.className = 'status ' + (data.gateState === 0 ? 'ok' : data.gateState === 1 ? 'bad' : 'warn');
      document.getElementById('weather').textContent = data.weather;
    }

    async function loadLog() {
      const res = await fetch('/log');
      const text = await res.text();
      document.getElementById('log').textContent = text || 'No log entries.';
    }

    document.getElementById('refresh').addEventListener('click', () => { loadStatus(); loadLog(); });
    document.getElementById('erase').addEventListener('click', async () => {
      if (!confirm('Erase the activity log?')) return;
      await fetch('/erase', { method: 'POST' });
      loadLog();
    });

    loadStatus();
    loadLog();
    setInterval(loadStatus, 5000);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  webServer.send_P(200, "text/html", INDEX_HTML);
}

void handleStatusJson() {
  String gateText = (gateState == 1) ? "GATE OPEN" : (gateState == 0) ? "GATE CLOSED" : "NO SIGNAL";
  String wifiText = (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
  String mqttText = mqttConnected ? "connected" : "disconnected";
  String timeText = getTimestampString();
  String ipText = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
  long uptimeSec = millis() / 1000;
  int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -127;
  String payload = "{";
  payload += "\"gate\":\"" + gateText + "\",";
  payload += "\"gateState\":" + String(gateState) + ",";
  payload += "\"wifi\":\"" + wifiText + "\",";
  payload += "\"mqtt\":\"" + mqttText + "\",";
  payload += "\"ip\":\"" + ipText + "\",";
  payload += "\"uptime\":" + String(uptimeSec) + ",";
  payload += "\"rssi\":" + String(rssi) + ",";
  payload += "\"weather\":\"" + weatherStr + "\",";
  payload += "\"time\":\"" + timeText + "\"";
  payload += "}";
  webServer.send(200, "application/json", payload);
}

void handleLog() {
  if (!SD.exists("/gate.log")) {
    webServer.send(200, "text/plain", "");
    return;
  }
  File file = SD.open("/gate.log", FILE_READ);
  if (!file) {
    webServer.send(500, "text/plain", "Failed to open log file");
    return;
  }
  String out;
  while (file.available()) {
    out += char(file.read());
  }
  file.close();
  webServer.send(200, "text/plain", out);
}

void handleEraseLog() {
  SD.remove("/gate.log");
  webServer.send(200, "text/plain", "OK");
}

void calibrateTouch() {
  uint16_t calData[5];

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20, 0);
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Touch corners as indicated");

  tft.setTextFont(1);
  tft.println();

  tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

  Serial.println();
  Serial.println("// Use this calibration code in setup():");
  Serial.print("  uint16_t calData[5] = { ");
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print(calData[i]);
    if (i < 4) Serial.print(", ");
  }
  Serial.println(" }; ");
  Serial.println("  tft.setTouch(calData);");
  Serial.println();

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("Calibration complete!");
  tft.println("Values sent to Serial.");

  delay(2000);
}
