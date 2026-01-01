/*
 * VERKEERSLICHT A - Met Web Interface
 * Normale, leesbare code
 */

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ============================================
// CONFIGURATIE
// ============================================

// WiFi - gebruik char arrays zodat we kunnen schrijven
char WIFI_SSID[32] = "WiFi-2.4-5800";
char WIFI_PASSWORD[64] = "wj6ahm5m2d9xn";

// MQTT
char MQTT_BROKER[64] = "test.mosquitto.org";
int MQTT_PORT = 1883;

// Timing (seconden)
int TIME_A_TO_B = 30;
int TIME_B_TO_A = 45;
int WAIT_TIME = 5;
int YELLOW_TIME = 3;
int HEARTBEAT_INTERVAL = 5;
int HEARTBEAT_TIMEOUT = 10;

// Hardware
const int PIN_RED = 21;
const int PIN_YELLOW = 22;
const int PIN_GREEN = 23;

// MQTT Topics
const char* TOPIC_STATUS_A = "traffic/lightA/status";
const char* TOPIC_STATUS_B = "traffic/lightB/status";
const char* TOPIC_HEARTBEAT_A = "traffic/lightA/heartbeat";
const char* TOPIC_HEARTBEAT_B = "traffic/lightB/heartbeat";

// ============================================
// GLOBALE VARIABELEN
// ============================================

enum State { GREEN, YELLOW, RED, ERROR };
State currentState = RED;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer webServer(80);
Preferences preferences;

unsigned long lastStateChange = 0;
unsigned long lastHeartbeatSent = 0;
unsigned long lastHeartbeatReceived = 0;
unsigned long lastBlink = 0;
bool blinkState = false;
bool systemReady = false;

// ============================================
// WEB INTERFACE HTML
// ============================================

const char WEB_PAGE[] = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Verkeerslicht A</title>
  <style>
    body { font-family: Arial; margin: 20px; }
    .box { background: white; padding: 20px; margin: 10px 0; border-radius: 5px; }
    input { width: 100%; padding: 8px; margin: 5px 0; }
    button { width: 100%; padding: 10px; background: blue; color: white; border: none; }
  </style>
</head>
<body>
  <div class="box">
    <h1>Verkeerslicht A</h1>
    <p>Status: <b>%STATUS%</b></p>
    <p>IP: %IP%</p>
  </div>
  
  <div class="box">
    <h2>Configuratie</h2>
    <form method="POST" action="/save">
      <label>WiFi SSID:</label>
      <input name="ssid" value="%SSID%">
      
      <label>WiFi Wachtwoord:</label>
      <input name="pass" type="password" value="%PASS%">
      
      <label>MQTT Broker:</label>
      <input name="broker" value="%BROKER%">
      
      <label>MQTT Port:</label>
      <input name="port" type="number" value="%PORT%">
      
      <label>A naar B Tijd (sec):</label>
      <input name="tab" type="number" value="%TAB%">
      
      <label>B naar A Tijd (sec):</label>
      <input name="tba" type="number" value="%TBA%">
      
      <label>Wacht Tijd (sec):</label>
      <input name="wait" type="number" value="%WAIT%">
      
      <label>Geel Licht Tijd (sec):</label>
      <input name="yellow" type="number" value="%YELLOW%">
      
      <button type="submit">Opslaan en Herstarten</button>
    </form>
  </div>
</body>
</html>
)";

// ============================================
// FUNCTIES
// ============================================

void loadSettings() {
  preferences.begin("traffic", true);  // read-only
  
  preferences.getString("ssid", WIFI_SSID, 32);
  preferences.getString("pass", WIFI_PASSWORD, 64);
  preferences.getString("broker", MQTT_BROKER, 64);
  
  TIME_A_TO_B = preferences.getInt("tab", 30);
  TIME_B_TO_A = preferences.getInt("tba", 45);
  WAIT_TIME = preferences.getInt("wait", 5);
  YELLOW_TIME = preferences.getInt("yellow", 3);
  
  preferences.end();
}

void saveSettings() {
  preferences.begin("traffic", false);
  
  preferences.putString("ssid", WIFI_SSID);
  preferences.putString("pass", WIFI_PASSWORD);
  preferences.putString("broker", MQTT_BROKER);
  preferences.putInt("tab", TIME_A_TO_B);
  preferences.putInt("tba", TIME_B_TO_A);
  preferences.putInt("wait", WAIT_TIME);
  preferences.putInt("yellow", YELLOW_TIME);
  
  preferences.end();
}

void handleWebRoot() {
  String html = WEB_PAGE;
  
  String status;
  if (currentState == GREEN) status = "GROEN";
  else if (currentState == YELLOW) status = "GEEL";
  else if (currentState == RED) status = "ROOD";
  else status = "ERROR";
  
  html.replace("%STATUS%", status);
  html.replace("%IP%", WiFi.localIP().toString());
  html.replace("%SSID%", WIFI_SSID);
  html.replace("%PASS%", WIFI_PASSWORD);
  html.replace("%BROKER%", MQTT_BROKER);
  html.replace("%PORT%", String(MQTT_PORT));
  html.replace("%TAB%", String(TIME_A_TO_B));
  html.replace("%TBA%", String(TIME_B_TO_A));
  html.replace("%WAIT%", String(WAIT_TIME));
  html.replace("%YELLOW%", String(YELLOW_TIME));
  
  webServer.send(200, "text/html", html);
}

void handleWebSave() {
  if (webServer.hasArg("ssid")) {
    String val = webServer.arg("ssid");
    val.toCharArray(WIFI_SSID, 32);
  }
  if (webServer.hasArg("pass")) {
    String val = webServer.arg("pass");
    val.toCharArray(WIFI_PASSWORD, 64);
  }
  if (webServer.hasArg("broker")) {
    String val = webServer.arg("broker");
    val.toCharArray(MQTT_BROKER, 64);
  }
  if (webServer.hasArg("tab")) {
    TIME_A_TO_B = webServer.arg("tab").toInt();
  }
  if (webServer.hasArg("tba")) {
    TIME_B_TO_A = webServer.arg("tba").toInt();
  }
  if (webServer.hasArg("wait")) {
    WAIT_TIME = webServer.arg("wait").toInt();
  }
  if (webServer.hasArg("yellow")) {
    YELLOW_TIME = webServer.arg("yellow").toInt();
  }
  
  saveSettings();
  
  webServer.send(200, "text/html", "<h1>Opgeslagen! ESP herstart...</h1>");
  delay(1000);
  ESP.restart();
}

void setState(State newState) {
  if (currentState == newState) return;
  
  currentState = newState;
  lastStateChange = millis();
  
  Serial.print("STATE: ");
  if (newState == GREEN) Serial.println("GREEN");
  else if (newState == YELLOW) Serial.println("YELLOW");
  else if (newState == RED) Serial.println("RED");
  else Serial.println("ERROR");
  
  // Publiceer status naar MQTT
  JsonDocument doc;
  doc["state"] = (newState == GREEN ? "GREEN" : 
                  newState == YELLOW ? "YELLOW" :
                  newState == RED ? "RED" : "ERROR");
  doc["light_id"] = "A";
  
  String output;
  serializeJson(doc, output);
  mqttClient.publish(TOPIC_STATUS_A, output.c_str(), true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  JsonDocument doc;
  deserializeJson(doc, message);
  
  // Heartbeat van B ontvangen
  if (strcmp(topic, TOPIC_HEARTBEAT_B) == 0) {
    lastHeartbeatReceived = millis();
    
    if (currentState == ERROR && systemReady) {
      setState(RED);
      delay(2000);
      setState(GREEN);
    }
  }
  
  // Status van B ontvangen
  else if (strcmp(topic, TOPIC_STATUS_B) == 0) {
    const char* state = doc["state"];
    
    if (strcmp(state, "OFFLINE") == 0 || strcmp(state, "ERROR") == 0) {
      setState(ERROR);
    }
    else if (strcmp(state, "GREEN") == 0 && currentState == RED) {
      lastStateChange = millis(); // Reset timer voor sync
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== VERKEERSLICHT A ===");
  
  loadSettings();
  
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  
  // WiFi verbinden
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWiFi: ");
  Serial.println(WiFi.localIP());
  
  // Web server starten
  webServer.on("/", handleWebRoot);
  webServer.on("/save", HTTP_POST, handleWebSave);
  webServer.begin();
  Serial.print("Web: http://");
  Serial.println(WiFi.localIP());
  
  // MQTT verbinden
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  
  while (!mqttClient.connected()) {
    if (mqttClient.connect("TrafficLightA")) {
      mqttClient.subscribe(TOPIC_STATUS_B);
      mqttClient.subscribe(TOPIC_HEARTBEAT_B);
      Serial.println("MQTT: OK");
    } else {
      delay(5000);
    }
  }
  
  lastHeartbeatReceived = millis();
  lastStateChange = millis();
  
  setState(GREEN);
  systemReady = true;
  Serial.println("READY\n");
}

void loop() {
  webServer.handleClient();
  
  if (!mqttClient.connected()) {
    if (mqttClient.connect("TrafficLightA")) {
      mqttClient.subscribe(TOPIC_STATUS_B);
      mqttClient.subscribe(TOPIC_HEARTBEAT_B);
    }
  }
  mqttClient.loop();
  
  // State machine
  unsigned long elapsed = millis() - lastStateChange;
  
  if (currentState == GREEN && elapsed >= TIME_A_TO_B * 1000) {
    setState(YELLOW);
  }
  else if (currentState == YELLOW && elapsed >= YELLOW_TIME * 1000) {
    setState(RED);
  }
  else if (currentState == RED && elapsed >= (TIME_B_TO_A + WAIT_TIME) * 1000) {
    setState(GREEN);
  }
  
  // LED's aansturen
  if (currentState == ERROR) {
    if (millis() - lastBlink >= 1000) {
      blinkState = !blinkState;
      lastBlink = millis();
    }
    digitalWrite(PIN_RED, blinkState);
    digitalWrite(PIN_YELLOW, blinkState);
    digitalWrite(PIN_GREEN, LOW);
  } else {
    digitalWrite(PIN_RED, currentState == RED);
    digitalWrite(PIN_YELLOW, currentState == YELLOW);
    digitalWrite(PIN_GREEN, currentState == GREEN);
  }
  
  // Heartbeat versturen
  if (millis() - lastHeartbeatSent >= HEARTBEAT_INTERVAL * 1000) {
    JsonDocument doc;
    doc["alive"] = true;
    doc["light_id"] = "A";
    
    String output;
    serializeJson(doc, output);
    mqttClient.publish(TOPIC_HEARTBEAT_A, output.c_str());
    
    lastHeartbeatSent = millis();
  }
  
  // Timeout check
  if (currentState != ERROR && systemReady) {
    if (millis() - lastHeartbeatReceived > HEARTBEAT_TIMEOUT * 1000) {
      setState(ERROR);
    }
  }
  
  delay(50);
}