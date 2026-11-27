#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC ADDRESS OF YOUR PLANE'S ESP32-S3 RECEIVER
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure for controls sent TO the plane
struct ControlMessage {
  int joystick1_X;
  int joystick1_Y;
  bool joystick1_Button;
  
  int joystick2_X;
  int joystick2_Y;         // Throttle axis
  bool payloadRelease;     // Triggered when Joystick 2 button is pressed
};

// Structure for telemetry received FROM the plane
struct TelemetryMessage {
  float pitch;
  float roll;
  float yaw;
  float altitude;
  float vspeed;
  float airspeed;
  float temp;
  float humidity;
  float pressure;
  float battery_v;
  int throttle;
  int gate_pos;
  float payload_mass;
  int rssi;
  int packet_loss;
  unsigned long frame_count;
};

ControlMessage outgoingData;
TelemetryMessage incomingData;
esp_now_peer_info_t peerInfo;

// Pin definitions (using safe ADC1 pins on ESP32-C3)
const int J1_X_PIN = 0;   // Left Joystick X axis (ADC1)
const int J1_Y_PIN = 1;   // Left Joystick Y axis (ADC1)
const int J1_SW_PIN = 6;  // Left Joystick Button (Digital)

const int J2_X_PIN = 2;   // Right Joystick X axis (ADC1)
const int J2_Y_PIN = 3;   // Right Joystick Y axis (ADC1 - Throttle)
const int J2_SW_PIN = 7;  // Right Joystick Button (Payload Release)

int failedPackets = 0;

// ESP-NOW Send Callback (Tracks transmission failures)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    failedPackets++;
  }
}

// ESP-NOW Receive Callback (Captures telemetry from plane & formats to JSON for Python)
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataBytes, int len) {
  if (len == sizeof(TelemetryMessage)) {
    memcpy(&incomingData, incomingDataBytes, sizeof(TelemetryMessage));
    
    // Print out JSON string for Python telemetry dashboard reader
    Serial.print("{");
    Serial.print("\"p\":"); Serial.print(incomingData.pitch, 1); Serial.print(",");
    Serial.print("\"r\":"); Serial.print(incomingData.roll, 1); Serial.print(",");
    Serial.print("\"y\":"); Serial.print(incomingData.yaw, 1); Serial.print(",");
    Serial.print("\"alt\":"); Serial.print(incomingData.altitude, 1); Serial.print(",");
    Serial.print("\"v\":"); Serial.print(incomingData.vspeed, 2); Serial.print(",");
    Serial.print("\"as\":"); Serial.print(incomingData.airspeed, 1); Serial.print(",");
    Serial.print("\"t\":"); Serial.print(incomingData.temp, 1); Serial.print(",");
    Serial.print("\"h\":"); Serial.print(incomingData.humidity, 1); Serial.print(",");
    Serial.print("\"pr\":"); Serial.print(incomingData.pressure, 1); Serial.print(",");
    Serial.print("\"bat\":"); Serial.print(incomingData.battery_v, 2); Serial.print(",");
    Serial.print("\"thr\":"); Serial.print(incomingData.throttle); Serial.print(",");
    Serial.print("\"gate\":"); Serial.print(incomingData.gate_pos); Serial.print(",");
    Serial.print("\"mass\":"); Serial.print(incomingData.payload_mass, 1); Serial.print(",");
    Serial.print("\"rssi\":"); Serial.print(incomingData.rssi); Serial.print(",");
    Serial.print("\"loss\":"); Serial.print(failedPackets); Serial.print(",");
    Serial.print("\"fc\":"); Serial.print(incomingData.frame_count);
    Serial.println("}");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set joystick button pins as inputs with internal pull-ups
  pinMode(J1_SW_PIN, INPUT_PULLUP);
  pinMode(J2_SW_PIN, INPUT_PULLUP);

  // Set ADC attenuation to 11dB for full 0V - 3.3V range
  analogSetAttenuation(ADC_11db);

  // Set Wi-Fi to Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback with explicit typecast for v3.x core compatibility
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peer (the plane)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP32-C3 Ground Control Initialized Successfully!");
}

void loop() {
  // Read analog values from Joysticks (0 to 4095 on 12-bit ADC)
  outgoingData.joystick1_X = analogRead(J1_X_PIN);
  outgoingData.joystick1_Y = analogRead(J1_Y_PIN);
  outgoingData.joystick1_Button = (digitalRead(J1_SW_PIN) == LOW);

  outgoingData.joystick2_X = analogRead(J2_X_PIN);
  outgoingData.joystick2_Y = analogRead(J2_Y_PIN);
  
  // Joystick 2 Button press triggers the payload gate release flag (Active LOW)
  outgoingData.payloadRelease = (digitalRead(J2_SW_PIN) == LOW);

  // Send control packet via ESP-NOW to the plane
  esp_now_send(broadcastAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));

  // ~20Hz update loop rate
  delay(50);
}
