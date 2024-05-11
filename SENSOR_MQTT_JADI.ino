#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <MQTT.h>

const int SENSOR_PIN = 4; // Arduino pin connected to DS18B20 sensor's DQ pin
const int led_pin = 23;   // Arduino pin connected to the LED

OneWire oneWire(SENSOR_PIN);         // Setup a oneWire instance
DallasTemperature tempSensor(&oneWire); // Pass oneWire to DallasTemperature library

float tempCelsius;    // Temperature in Celsius
bool ledState = false; // LED state (initially off)

// WiFi settings
#define WIFI_SSID             "Ajam"
#define WIFI_PASSWORD         "ajam1234"

// MQTT settings
#define MQTT_HOST             "broker.hivemq.com"
#define MQTT_PREFIX_TOPIC     "csm3313_umt/group10"
#define MQTT_PUBLISH_TOPIC1   "/watertemp"
#define MQTT_PUBLISH_TOPIC2   "/led01" // MQTT topic for LED
#define MQTT_SUBSCRIBE_TOPIC2 "/led01" // MQTT topic for LED
#define MQTT_PORT             1883

WiFiClient net;
MQTTClient mqtt(1024);
unsigned long lastMillis = 0;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi '" + String(WIFI_SSID) + "' ...");

  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // while wifi not connected yet, print '.'
  // then after it connected, get out of the loop
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //print a new line, then print WiFi connected and the IP address
  Serial.println("");
  Serial.println("WiFi connected");
  // Print the IP address
  Serial.println(WiFi.localIP());
}

void messageReceived(String &topic, String &payload) {
  Serial.println("Received message from topic: " + topic + " - payload: " + payload);

  if (topic.equals(String(MQTT_PREFIX_TOPIC) + String(MQTT_SUBSCRIBE_TOPIC2))) {
    // Received MQTT message for LED control
    if (payload.equals("ON")) {
      ledState = true;
      digitalWrite(led_pin, HIGH); // Turn on the LED
      Serial.println("LED ON");
    } else if (payload.equals("OFF")) {
      ledState = false;
      digitalWrite(led_pin, LOW); // Turn off the LED
      Serial.println("LED OFF");
    }
  }
}

void connectToMqttBroker() {
  Serial.print("Connecting to '" + String(MQTT_HOST) + "' ...");

  mqtt.begin(MQTT_HOST, net);
  // Add more subscriptions as needed
  mqtt.subscribe(MQTT_SUBSCRIBE_TOPIC2, messageReceived);

  while (!mqtt.connect("arduino-client")) {
    Serial.print(".");
    delay(500);
  }

  Serial.println(" connected!");

  // Subscribe to additional topics if needed
  mqtt.subscribe(MQTT_SUBSCRIBE_TOPIC2, messageReceived);
}

void setup() {
  Serial.begin(115200); // Initialize serial
  tempSensor.begin();    // Initialize the sensor
  pinMode(led_pin, OUTPUT); // Set the LED pin as an OUTPUT

  connectToWiFi();
  connectToMqttBroker();
}

void loop() {
  // Temperature Sensing
  tempSensor.requestTemperatures();             // Send the command to get temperatures
  tempCelsius = tempSensor.getTempCByIndex(0);  // Read temperature in Celsius

  Serial.print("Temperature: ");
  Serial.print(tempCelsius);    // Print the temperature in Celsius
  Serial.println("°C");

  // Publish temperature to MQTT
  Serial.print("Publishing to MQTT topic: ");
  Serial.println(String(MQTT_PREFIX_TOPIC) + String(MQTT_PUBLISH_TOPIC1));
  mqtt.publish(String(MQTT_PREFIX_TOPIC) + String(MQTT_PUBLISH_TOPIC1), String(tempCelsius));

  // Handle MQTT messages for LED control
  mqtt.loop();

  // Add your additional code here to perform actions while the LED is on
  // For example, you can add more sensor readings or other tasks

  delay(5000); // Wait for 5 seconds before the next iteration

  // Handle LED state
  if (ledState) {
    // Add your additional code here to perform actions while the LED is on
    // For example, you can add more sensor readings or other tasks
  }

  delay(5000); // Wait for 5 seconds before the next iteration
}
