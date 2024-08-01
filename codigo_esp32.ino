#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

// Pines y configuración del DHT22
#define DHTPIN 15
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// Pines y configuración de los sensores de humedad del suelo
const int humsueloPin1 = 33;
const int humsueloPin2 = 32;

// Pines de los relés
const int releSueloPin = 18;
const int releNodoRedPin = 12;
const int releFocoPin = 13;     

// Configuración de la red WiFi
const char* ssid = "GreenHouse";
const char* password = "sistemas2002";

// Configuración de los servidores MQTT
const char* mqtt_server1 = "192.168.0.100";
const int mqtt_port1 = 1887;
const char* mqtt_user1 = "greenhouse";
const char* mqtt_password1 = "sistemas2002";

const char* mqtt_server2 = "13.58.67.42";
const int mqtt_port2 = 1883;
const char* mqtt_user2 = "greenhouse1";
const char* mqtt_password2 = "sistemas2002";

// Definir topics para cada sensor
const char* topic_temp = "miniinvernadero/temperatura";
const char* topic_hum_aire = "miniinvernadero/humedad_aire";
const char* topic_hum_suelo1 = "miniinvernadero/humedad_suelo";
const char* topic_hum_suelo2 = "miniinvernadero/humedad_suelo2";
const char* topic_rele = "miniinvernadero/rele";
const char* topic_control = "esp32/control";
const char* topic_duration = "esp32/control/duration";
const char* topic_foco = "miniinvernadero/foco";

// Crear clientes WiFi y clientes MQTT
WiFiClient espClient1;
WiFiClient espClient2;
PubSubClient client1(espClient1);
PubSubClient client2(espClient2);

bool cloudConnected = true;
unsigned long lastReconnectAttempt = 0;
unsigned long lastSensorRead = 0;
unsigned long lastPublish = 0;
unsigned long lastStateCheck = 0;
const unsigned long stateCheckInterval = 5000; // Comprobar cada 5 segundos

float lastTemp = 0;
float lastHumAire = 0;
int lastHumSuelo1 = 0;
int lastHumSuelo2 = 0;

int lastMQTT1State = -1;
int lastMQTT2State = -1;

void setup() {
  Serial.begin(115200);
  delay(1000); // Dar tiempo para que el puerto serial se inicialice

  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  Serial.println(reason);

  analogReadResolution(12);
  pinMode(humsueloPin1, INPUT);
  pinMode(humsueloPin2, INPUT);

  dht.begin();

  pinMode(releSueloPin, OUTPUT);
  digitalWrite(releSueloPin, LOW);

  pinMode(releNodoRedPin, OUTPUT);
  digitalWrite(releNodoRedPin, LOW);

  pinMode(releFocoPin, OUTPUT);
  digitalWrite(releFocoPin, LOW);

  setup_wifi();

  client1.setServer(mqtt_server1, mqtt_port1);
  client1.setCallback(callback);
  client1.setKeepAlive(60);
  
  client2.setServer(mqtt_server2, mqtt_port2);
  client2.setCallback(callback);
  client2.setKeepAlive(60);

  // Desactivar el WDT existente
  esp_task_wdt_deinit();

  // Configurar watchdog con un tiempo de espera más largo
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 60000, // 60 segundos
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
}

void setup_wifi() {
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("Dirección IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("Fallo en la conexión WiFi");
  }
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexión WiFi perdida. Reconectando...");
    WiFi.disconnect();
    WiFi.reconnect();
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconexión WiFi exitosa");
    } else {
      Serial.println("Fallo en la reconexión WiFi");
    }
  }
}

bool reconnect(PubSubClient &client, const char* clientID, const char* username, const char* password) {
  if (!client.connected()) {
    Serial.print("Conectando al servidor MQTT...");
    if (client.connect(clientID, username, password)) {
      Serial.println("Conectado");
      client.subscribe(topic_rele);
      client.subscribe(topic_control);
      client.subscribe(topic_duration);
      client.subscribe(topic_foco);
      return true;
    } else {
      Serial.print("Error de conexión, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 30 segundos");
      return false;
    }
  }
  return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;

  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == topic_rele) {
    if (message == "ON") {
      digitalWrite(releNodoRedPin, HIGH);
      Serial.println("Relé Node-RED encendido");
    } else if (message == "OFF") {
      digitalWrite(releNodoRedPin, LOW);
      Serial.println("Relé Node-RED apagado");
    }
  } else if (String(topic) == topic_control) {
    if (message == "RESTART") {
      ESP.restart();
    }
  } else if (String(topic) == topic_duration) {
    int duration = message.toInt();
    Serial.print("Encendiendo la bomba por ");
    Serial.print(duration);
    Serial.println(" segundos");
    digitalWrite(releNodoRedPin, HIGH);
    delay(duration * 1000);
    digitalWrite(releNodoRedPin, LOW);
  } else if (String(topic) == topic_foco) {
    if (message == "ON") {
      digitalWrite(releFocoPin, HIGH);
      Serial.println("Foco encendido");
    } else if (message == "OFF") {
      digitalWrite(releFocoPin, LOW);
      Serial.println("Foco apagado");
    }
  }
}

void checkAndPrintMQTTState() {
  int currentMQTT1State = client1.state();
  int currentMQTT2State = client2.state();
  
  if (currentMQTT1State != lastMQTT1State) {
    Serial.printf("Estado MQTT 1 cambiado a: %d\n", currentMQTT1State);
    lastMQTT1State = currentMQTT1State;
  }
  
  if (currentMQTT2State != lastMQTT2State) {
    Serial.printf("Estado MQTT 2 cambiado a: %d\n", currentMQTT2State);
    lastMQTT2State = currentMQTT2State;
  }
}

void loop() {
  esp_task_wdt_reset(); // Alimentar el watchdog
  
  checkWiFiConnection();

  unsigned long currentMillis = millis();
  if (currentMillis - lastStateCheck >= stateCheckInterval) {
    checkAndPrintMQTTState();
    lastStateCheck = currentMillis;
  }

  if (!client1.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 30000) {
      lastReconnectAttempt = now;
      if (reconnect(client1, "ESP32Client1", mqtt_user1, mqtt_password1)) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client1.loop();
  }

  if (cloudConnected && !client2.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 30000) {
      lastReconnectAttempt = now;
      if (reconnect(client2, "ESP32Client2", mqtt_user2, mqtt_password2)) {
        lastReconnectAttempt = 0;
      }
    }
  } else if (cloudConnected) {
    client2.loop();
  }

  unsigned long now = millis();
  if (now - lastSensorRead > 500) {
    lastSensorRead = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    int valHumsuelo1 = analogRead(humsueloPin1);
    int valHumsuelo2 = analogRead(humsueloPin2);

    if (!isnan(h) && !isnan(t) && (t != lastTemp || h != lastHumAire)) {
      lastTemp = t;
      lastHumAire = h;
      char tempString[8];
      char humAireString[8];
      dtostrf(t, 1, 2, tempString);
      dtostrf(h, 1, 2, humAireString);
      client1.publish(topic_temp, tempString);
      client2.publish(topic_temp, tempString);
      client1.publish(topic_hum_aire, humAireString);
      client2.publish(topic_hum_aire, humAireString);
    }

    int humsueloPercentage1 = map(valHumsuelo1, 4095, 1303, 0, 100);
    int humsueloPercentage2 = map(valHumsuelo2, 4095, 1303, 0, 100);

    if (humsueloPercentage1 != lastHumSuelo1) {
      lastHumSuelo1 = humsueloPercentage1;
      char humsueloString1[8];
      dtostrf(humsueloPercentage1, 1, 2, humsueloString1);
      client1.publish(topic_hum_suelo1, humsueloString1);
      client2.publish(topic_hum_suelo1, humsueloString1);
    }

    if (humsueloPercentage2 != lastHumSuelo2) {
      lastHumSuelo2 = humsueloPercentage2;
      char humsueloString2[8];
      dtostrf(humsueloPercentage2, 1, 2, humsueloString2);
      client1.publish(topic_hum_suelo2, humsueloString2);
      client2.publish(topic_hum_suelo2, humsueloString2);
    }

    if (humsueloPercentage1 < 50 || humsueloPercentage2 < 50 ) {
      digitalWrite(releSueloPin, HIGH);
    } else {
      digitalWrite(releSueloPin, LOW);
    }
  }

  delay(100);
}

