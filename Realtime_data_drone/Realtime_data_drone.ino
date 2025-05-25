/*
Author: Pham Thanh Bien
Date: 2025-05-20
Description: ESP8266 MQTT client to send telemetry data Drone to ThingsBoard
Hardware: ESP8266
License: This code is licensed under the MIT License.
Copyright (c) 2025 Pham Thanh Bien
*/  
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "Bieniphone";
const char* password = "24445va6";
// const char* ssid = "SonThanh";
// const char* password = "0912480208";
// const char* ssid = "Nhan";
// const char* password = "12345678";
const char* token = "LjAXHMVcmIzujOBJ8mAz"; 
const char* mqtt_server = "demo.thingsboard.io";

WiFiClient espClient;
PubSubClient client(espClient);

#define LED_PIN 0 // GPIO0

// UART STM32
#define STM32_RX 3
#define STM32_TX 1
HardwareSerial stmSerial(1);
String inputBuffer = "";
bool wasConnected = true;


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connect to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Connect successful!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("RPC receive from topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.write(payload, length);
  Serial.println();

  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, payload, length);

  if (err) {
    Serial.print("Lỗi parse JSON: ");
    Serial.println(err.f_str());
    return;
  }

  const char* method = doc["method"];
  bool params = doc["params"];

  Serial.print("Method: ");
  Serial.println(method);
  Serial.print("Params: ");
  Serial.println(params);

 if (strcmp(method, "ledState") == 0) {
    if (params) {
      digitalWrite(LED_PIN, HIGH); 
      Serial.println("LED: ON");
    } else {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED: OFF");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connect to MQTT...");
    if (client.connect("ESP8266Client", token, NULL)) {
      Serial.println("Successful!");
      client.subscribe("v1/devices/me/rpc/request/+");
    } else {
      Serial.print("Error: ");
      Serial.print(client.state());
      Serial.println(" - try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  stmSerial.begin(115200, SERIAL_8N1, SERIAL_FULL, STM32_RX, STM32_TX); 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      parseAndSend(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += ch;
    }
  }
}


void parseAndSend(String data) {
  float roll = 0, pitch = 0, kp = 0, ki = 0, kd = 0;
  int th1 = 0, th2 = 0, th3 = 0, th4 = 0;
  int commaIndex = data.indexOf(',');

  // if (commaIndex > 0 && commaIndex < data.length() - 1) {
  //   roll = data.substring(0, commaIndex).toFloat();
  //   pitch = data.substring(commaIndex + 1).toFloat();

  //   // Gửi lên ThingsBoard (JSON payload)
  //   String payload = "{\"roll\":" + String(roll, 2) + ",\"pitch\":" + String(pitch, 2) + "}";
  //   client.publish("v1/devices/me/telemetry", payload.c_str());
  //   Serial.println("Sent: " + payload);
  // } 

   int result = sscanf(data.c_str(), "%f,%f,%d,%d,%d,%d,%f,%f,%f", &roll, &pitch, &th1, &th2, &th3, &th4, &kp, &ki, &kd);
  
   if (result == 9) {  
    //JSON payload
    String payload = "{";
    payload += "\"roll\":" + String(roll, 2) + ",";
    payload += "\"pitch\":" + String(pitch, 2) + ",";
    payload += "\"th1\":" + String(th1) + ",";
    payload += "\"th2\":" + String(th2) + ",";
    payload += "\"th3\":" + String(th3) + ",";
    payload += "\"th4\":" + String(th4) + ",";
    payload += "\"kp\":" + String(kp) + ",";
    payload += "\"ki\":" + String(ki) + ",";
    payload += "\"kd\":" + String(kd);
    payload += "}";
     client.publish("v1/devices/me/telemetry", payload.c_str());
    Serial.println("Sent: " + payload);
  }
  else {
    Serial.println("Parse error: " + data);
  }

  int rssi = WiFi.RSSI();  
  // String payload = "{\"wifi_signal\":" + String(rssi) + "}";
  // client.publish("v1/devices/me/telemetry", payload.c_str());

  // Read ADC
  int adcValue = analogRead(A0);  // 0 - 1023
  float vADC = adcValue * (3.3 / 1023.0);  
  float batteryVoltage = vADC * (48.05 / 9.75) * 0.9622;  // R1 = 39k, R2 = 10k

  // Create JSON 
  // String payload = "{\"battery_voltage\":" + String(batteryVoltage, 2) + "}";
  // Serial.println(payload);
  String payload = "{";
payload += "\"wifi_signal\":" + String(rssi) + ",";
payload += "\"battery_voltage\":" + String(batteryVoltage, 2);
payload += "}";

  client.publish("v1/devices/me/telemetry", payload.c_str());
}
