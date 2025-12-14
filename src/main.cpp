/*
 * VERKEERSLICHT A - Draadloos Verkeerslichtsysteem
 * ESP32 met MQTT communicatie
 * 
 * Hardware: 
 * - Rode LED op GPIO 25
 * - Gele LED op GPIO 26  
 * - Groene LED op GPIO 27
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ============================================
// CONFIGURATIE - PAS DEZE WAARDEN AAN
// ============================================

// WiFi instellingen
const char* WIFI_SSID = "WiFi-2.4-5800";
const char* WIFI_PASSWORD = "wj6ahm5m2d9xn";

// MQTT Broker instellingen
const char* MQTT_BROKER = "test.mosquitto.org";   // Publieke broker
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "TrafficLightA";     // Unieke ID voor dit verkeerslicht

// Verkeerslicht identificatie
const char* LIGHT_ID = "A";                       // Dit is verkeerslicht A

// Timing configuratie (in seconden)
const int TIME_A_TO_B = 30;        // Tijd dat verkeer van A naar B mag rijden
const int TIME_B_TO_A = 45;        // Tijd dat verkeer van B naar A mag rijden  
const int WAIT_TIME = 5;           // Wachttijd tussen richtingwisselingen
const int YELLOW_TIME = 3;         // Duur van geel licht (vast volgens opgave)

// Communicatie instellingen
const int HEARTBEAT_INTERVAL = 5;  // Heartbeat elke 5 seconden
const int HEARTBEAT_TIMEOUT = 10;  // Timeout na 10 seconden geen heartbeat
const int ERROR_BLINK_INTERVAL = 1; // Knipperinterval in ERROR mode (1 sec)

// ============================================
// HARDWARE PINNEN
// ============================================
const int PIN_RED = 21;
const int PIN_YELLOW = 22;
const int PIN_GREEN = 23;

// ============================================
// MQTT TOPICS
// ============================================
const char* TOPIC_STATUS_A = "traffic/lightA/status";
const char* TOPIC_STATUS_B = "traffic/lightB/status";
const char* TOPIC_HEARTBEAT_A = "traffic/lightA/heartbeat";
const char* TOPIC_HEARTBEAT_B = "traffic/lightB/heartbeat";

// ============================================
// STATE MACHINE
// ============================================
enum State {
  STATE_GREEN,
  STATE_YELLOW,
  STATE_RED,
  STATE_ERROR
};

State currentState = STATE_RED;  // Start in RED, wacht op synchronisatie
State previousState = STATE_RED;

// ============================================
// GLOBALE VARIABELEN
// ============================================
WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastStateChange = 0;
unsigned long lastHeartbeatSent = 0;
unsigned long lastHeartbeatReceived = 0;
unsigned long lastBlinkToggle = 0;

bool otherLightOnline = false;
bool errorBlinkState = false;
bool systemReady = false;

// ============================================
// FUNCTIE DECLARATIES
// ============================================
void setupWiFi();
void setupMQTT();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishStatus(State state);
void publishHeartbeat();
void updateLEDs();
void handleStateMachine();
void switchState(State newState);
void checkHeartbeatTimeout();

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=================================");
  Serial.println("VERKEERSLICHT A - SYSTEEM START");
  Serial.println("=================================\n");
  
  // Configureer LED pins
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  
  // Start met alle LED's uit
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_YELLOW, LOW);
  digitalWrite(PIN_GREEN, LOW);
  
  // Verbind met WiFi
  setupWiFi();
  
  // Configureer MQTT
  setupMQTT();
  
  // Start timers
  lastHeartbeatReceived = millis();
  lastStateChange = millis();
  
  // Verkeerslicht A start als MASTER in GREEN status
  Serial.println("Verkeerslicht A start als MASTER (GREEN)");
  switchState(STATE_GREEN);
  
  systemReady = true;
  Serial.println("\nSysteem klaar!\n");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Controleer WiFi verbinding
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi verbinding verloren! Probeer opnieuw...");
    setupWiFi();
  }
  
  // Controleer MQTT verbinding
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  // Update state machine
  handleStateMachine();
  
  // Update LED's
  updateLEDs();
  
  // Verstuur heartbeat
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatSent >= HEARTBEAT_INTERVAL * 1000) {
    publishHeartbeat();
    lastHeartbeatSent = currentMillis;
  }
  
  // Controleer heartbeat timeout
  checkHeartbeatTimeout();
  
  delay(50); // Kleine delay voor stabiliteit
}

// ============================================
// WIFI SETUP
// ============================================
void setupWiFi() {
  Serial.print("Verbinden met WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi verbonden!");
    Serial.print("IP adres: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi verbinding mislukt!");
    switchState(STATE_ERROR);
  }
}

// ============================================
// MQTT SETUP
// ============================================
void setupMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  
  // Configureer buffer grootte voor JSON berichten
  mqttClient.setBufferSize(512);
  
  reconnectMQTT();
}

// ============================================
// MQTT RECONNECT
// ============================================
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Verbinden met MQTT broker...");
    
    // Configureer Last Will bericht
    String lastWillPayload = "{\"state\":\"OFFLINE\",\"timestamp\":" + 
                            String(millis()) + ",\"light_id\":\"" + 
                            String(LIGHT_ID) + "\"}";
    
    // Probeer verbinding met Last Will
    if (mqttClient.connect(MQTT_CLIENT_ID, 
                          TOPIC_STATUS_A,      // Last Will topic
                          1,                    // QoS 1
                          true,                 // Retain
                          lastWillPayload.c_str())) {
      
      Serial.println(" Verbonden!");
      
      // Abonneer op status en heartbeat van verkeerslicht B
      mqttClient.subscribe(TOPIC_STATUS_B, 1);
      mqttClient.subscribe(TOPIC_HEARTBEAT_B, 1);
      
      Serial.println("Geabonneerd op:");
      Serial.print("  - ");
      Serial.println(TOPIC_STATUS_B);
      Serial.print("  - ");
      Serial.println(TOPIC_HEARTBEAT_B);
      
      // Publiceer initiele status
      publishStatus(currentState);
      
    } else {
      Serial.print(" Mislukt! Error code: ");
      Serial.println(mqttClient.state());
      Serial.println("Nieuwe poging over 5 seconden...");
      
      switchState(STATE_ERROR);
      delay(5000);
    }
  }
}

// ============================================
// MQTT CALLBACK - Ontvang berichten
// ============================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Converteer payload naar string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("\n--- Bericht ontvangen ---");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Bericht: ");
  Serial.println(message);
  
  // Parse JSON
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Verwerk HEARTBEAT berichten
  if (strcmp(topic, TOPIC_HEARTBEAT_B) == 0) {
    lastHeartbeatReceived = millis();
    otherLightOnline = true;
    
    Serial.println("Heartbeat van B ontvangen - OK");
    
    // Als we in ERROR waren en heartbeat komt terug, herstel
    if (currentState == STATE_ERROR && systemReady) {
      Serial.println("Communicatie hersteld! Veilig herstarten...");
      switchState(STATE_RED);
      delay(2000); // Wacht even voor veiligheid
      switchState(STATE_GREEN); // A start weer als master
    }
  }
  
  // Verwerk STATUS berichten
  else if (strcmp(topic, TOPIC_STATUS_B) == 0) {
    const char* state = doc["state"];
    
    Serial.print("Status van B: ");
    Serial.println(state);
    
    // Als andere verkeerslicht offline is, ga naar ERROR
    if (strcmp(state, "OFFLINE") == 0) {
      Serial.println("Verkeerslicht B is OFFLINE! -> ERROR mode");
      switchState(STATE_ERROR);
      otherLightOnline = false;
    }
    // Als andere verkeerslicht in ERROR is, ga ook naar ERROR
    else if (strcmp(state, "ERROR") == 0) {
      Serial.println("Verkeerslicht B is in ERROR! -> ERROR mode");
      switchState(STATE_ERROR);
    }
    // Als B groen wordt EN A is rood, reset timer voor synchronisatie
    else if (strcmp(state, "GREEN") == 0 && currentState == STATE_RED) {
      Serial.println("✓ B is groen geworden - A reset timer");
      lastStateChange = millis();
    }
  }
  
  Serial.println("-------------------------\n");
}

// ============================================
// PUBLICEER STATUS
// ============================================
void publishStatus(State state) {
  JsonDocument doc;
  
  // Converteer state naar string
  const char* stateStr;
  switch(state) {
    case STATE_GREEN:  stateStr = "GREEN";  break;
    case STATE_YELLOW: stateStr = "YELLOW"; break;
    case STATE_RED:    stateStr = "RED";    break;
    case STATE_ERROR:  stateStr = "ERROR";  break;
    default:           stateStr = "UNKNOWN"; break;
  }
  
  doc["state"] = stateStr;
  doc["timestamp"] = millis();
  doc["light_id"] = LIGHT_ID;
  
  // Serialiseer naar JSON string
  String output;
  serializeJson(doc, output);
  
  // Publiceer met QoS 1 en retain
  if (mqttClient.publish(TOPIC_STATUS_A, output.c_str(), true)) {
    Serial.print("Status gepubliceerd: ");
    Serial.println(output);
  } else {
    Serial.println("Status publicatie mislukt!");
  }
}

// ============================================
// PUBLICEER HEARTBEAT
// ============================================
void publishHeartbeat() {
  JsonDocument doc;
  
  doc["alive"] = true;
  doc["timestamp"] = millis();
  doc["light_id"] = LIGHT_ID;
  
  String output;
  serializeJson(doc, output);
  
  if (mqttClient.publish(TOPIC_HEARTBEAT_A, output.c_str())) {
    Serial.print("♥ Heartbeat verzonden: ");
    Serial.println(output);
  }
}

// ============================================
// CONTROLEER HEARTBEAT TIMEOUT
// ============================================
void checkHeartbeatTimeout() {
  unsigned long currentMillis = millis();
  
  // Alleen checken als we niet in ERROR zijn en systeem is klaar
  if (currentState != STATE_ERROR && systemReady) {
    if (currentMillis - lastHeartbeatReceived > HEARTBEAT_TIMEOUT * 1000) {
      Serial.println("\n!!! HEARTBEAT TIMEOUT - Geen communicatie met B !!!");
      switchState(STATE_ERROR);
      otherLightOnline = false;
    }
  }
}

// ============================================
// STATE MACHINE HANDLER
// ============================================
void handleStateMachine() {
  if (currentState == STATE_ERROR) {
    // In ERROR mode doen we niets met timing
    return;
  }
  
  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - lastStateChange;
  
  switch(currentState) {
    case STATE_GREEN:
      // Na TIME_A_TO_B seconden, wissel naar YELLOW
      if (elapsed >= TIME_A_TO_B * 1000) {
        switchState(STATE_YELLOW);
      }
      break;
      
    case STATE_YELLOW:
      // Na YELLOW_TIME seconden, wissel naar RED
      if (elapsed >= YELLOW_TIME * 1000) {
        switchState(STATE_RED);
      }
      break;
      
    case STATE_RED:
      // Blijf in RED tijdens TIME_B_TO_A + WAIT_TIME
      // Dan wissel terug naar GREEN
      if (elapsed >= (TIME_B_TO_A + WAIT_TIME) * 1000) {
        switchState(STATE_GREEN);
      }
      break;
  }
}

// ============================================
// WISSEL NAAR NIEUWE STATE
// ============================================
void switchState(State newState) {
  if (currentState == newState) {
    return; // Geen verandering
  }
  
  previousState = currentState;
  currentState = newState;
  lastStateChange = millis();
  
  // Print state change
  Serial.print("\n>>> STATE CHANGE: ");
  switch(previousState) {
    case STATE_GREEN:  Serial.print("GREEN");  break;
    case STATE_YELLOW: Serial.print("YELLOW"); break;
    case STATE_RED:    Serial.print("RED");    break;
    case STATE_ERROR:  Serial.print("ERROR");  break;
  }
  Serial.print(" -> ");
  switch(currentState) {
    case STATE_GREEN:  Serial.println("GREEN");  break;
    case STATE_YELLOW: Serial.println("YELLOW"); break;
    case STATE_RED:    Serial.println("RED");    break;
    case STATE_ERROR:  Serial.println("ERROR");  break;
  }
  
  // Publiceer nieuwe status
  publishStatus(currentState);
}

// ============================================
// UPDATE LED's
// ============================================
void updateLEDs() {
  switch(currentState) {
    case STATE_GREEN:
      digitalWrite(PIN_RED, LOW);
      digitalWrite(PIN_YELLOW, LOW);
      digitalWrite(PIN_GREEN, HIGH);
      break;
      
    case STATE_YELLOW:
      digitalWrite(PIN_RED, LOW);
      digitalWrite(PIN_YELLOW, HIGH);
      digitalWrite(PIN_GREEN, LOW);
      break;
      
    case STATE_RED:
      digitalWrite(PIN_RED, HIGH);
      digitalWrite(PIN_YELLOW, LOW);
      digitalWrite(PIN_GREEN, LOW);
      break;
      
    case STATE_ERROR:
      // Knipperende rode + gele LED (1 seconde interval)
      unsigned long currentMillis = millis();
      if (currentMillis - lastBlinkToggle >= ERROR_BLINK_INTERVAL * 1000) {
        errorBlinkState = !errorBlinkState;
        lastBlinkToggle = currentMillis;
      }
      
      digitalWrite(PIN_RED, errorBlinkState ? HIGH : LOW);
      digitalWrite(PIN_YELLOW, errorBlinkState ? HIGH : LOW);
      digitalWrite(PIN_GREEN, LOW);
      break;
  }
}
