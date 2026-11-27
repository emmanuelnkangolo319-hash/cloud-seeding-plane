#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Sensor & I2C Setup
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

#define I2C_SDA 10
#define I2C_SCL 11

#define DHTPIN 21     
#define DHTTYPE DHT22   
DHT dht(DHTPIN, DHTTYPE);
sensors_event_t a, g, temp_event;

// Servo definitions and pins
Servo servo1;       // Control Surface 1
Servo servo2;       // Control Surface 2
Servo servoGate;    // Salt Dispenser Gate Servo

const int SERVO1_PIN = 4;
const int SERVO2_PIN = 5;
const int GATE_SERVO_PIN = 37; // 3rd Servo for payload release at Pin 38

// =========================================================================
// FUTURE ESC / BRUSHLESS MOTOR SETUP (Uncomment when wiring your ESC):
// Servo escMotor;
// const int ESC_PIN = 6; // Choose an available free GPIO on your S3
// =========================================================================

// Identical data structure matching the Ground Station controls
struct ControlMessage {
  int joystick1_X;
  int joystick1_Y;
  bool joystick1_Button;
  
  int joystick2_X;
  int joystick2_Y;         // Throttle axis
  bool payloadRelease;     // Triggered when Joystick 2 button is pressed
};

// Identical telemetry data structure sent back to Ground Station
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

ControlMessage incomingReadings;
TelemetryMessage outgoingTelemetry;

uint8_t groundStationAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Replace with your Ground Station MAC if unicast is preferred, or broadcast
esp_now_peer_info_t peerInfo;

// Failsafe Safety Parameters
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500; // Milliseconds before triggering signal-loss failsafe
unsigned long frameCounter = 0;
float currentPayloadMass = 150.0; // Starting salt payload in grams

// ESP-NOW Receive Callback (Modern ESP-IDF / Arduino core signature)
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(ControlMessage)) return;

  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));

  // Refresh failsafe countdown timer on every valid received packet
  lastPacketTime = millis();

  // Map joystick values (0 - 4095) to safe structural servo angles (0 - 180 degrees)
  int angle1 = constrain(map(incomingReadings.joystick1_X, 0, 4095, 0, 180), 0, 180);
  int angle2 = constrain(map(incomingReadings.joystick2_X, 0, 4095, 0, 180), 0, 180);

  // Actuate control surfaces
  servo1.write(angle1);
  servo2.write(angle2);

  // Payload Release Gate Control via 3rd servo on Pin 38
  if (incomingReadings.payloadRelease) {
    servoGate.write(90); // Open position to dispense salt
    if (currentPayloadMass > 0.0) {
      currentPayloadMass = max(0.0, currentPayloadMass - 0.5); // Deplete payload while open
    }
  } else {
    servoGate.write(0);  // Closed position
  }

  // =========================================================================
  // BRUSHLESS MOTOR ESC CONTROL LOGIC:
  // int throttlePWM = map(incomingReadings.joystick2_Y, 0, 4095, 1000, 2000);
  // escMotor.writeMicroseconds(throttlePWM);
  // =========================================================================
}

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("ESP32-S3 Flight Controller Booting...");

  // Attach control surfaces and payload gate servo
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servoGate.attach(GATE_SERVO_PIN);

  // Set initial neutral posture on startup
  servo1.write(90);
  servo2.write(90);
  servoGate.write(0); // Gate closed

  // Initialize sensors
  dht.begin();
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bmp.begin()) {
    Serial.println("Warning: BMP180 sensor not detected!");
  }
  if (!mpu.begin()) {
    Serial.println("Warning: MPU6050 sensor not detected!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Initialize Wi-Fi in Station mode for ESP-NOW radio layer
  WiFi.mode(WIFI_STA);
  delay(500);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW on Flight Controller!");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Register ground station peer for telemetry replies
  memcpy(peerInfo.peer_addr, groundStationAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Print hardware MAC address
  Serial.print("Flight Controller MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // --- SIGNAL LOSS FAILSAFE SAFETY CHECK ---
  if (millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
    servo1.write(90); 
    servo2.write(90); 
    servoGate.write(0); // Close gate on signal loss
    // escMotor.writeMicroseconds(1000); // Cut throttle if ESC is active
  }

  // Telemetry collection & broadcast loop (~10Hz)
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime >= 100) {
    lastTelemetryTime = millis();
    frameCounter++;

    // Read Sensors
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    float pressure = bmp.readPressure(); // Pascals
    float altitude = bmp.readAltitude(); // Meters
    
    // Read MPU6050 IMU Data
    sensors_event_t a, g, temp_event;
    mpu.getEvent(&a, &g, &temp_event);

    // Populate Telemetry Message Structure
    outgoingTelemetry.pitch = a.acceleration.y * 10.0; // Estimated pitch approximation from accel
    outgoingTelemetry.roll = a.acceleration.x * 10.0;  // Estimated roll approximation from accel
    outgoingTelemetry.yaw = 0.0;
    outgoingTelemetry.altitude = isnan(altitude) ? 0.0 : altitude;
    outgoingTelemetry.vspeed = 0.0; // Can be derived from derivative of barometric altitude
    outgoingTelemetry.airspeed = 13.0; // Estimated cruise airspeed
    outgoingTelemetry.temp = isnan(temperature) ? 18.5 : temperature;
    outgoingTelemetry.humidity = isnan(humidity) ? 82.0 : humidity;
    outgoingTelemetry.pressure = isnan(pressure) ? 1013.25 : (pressure / 100.0); // Convert Pa to hPa
    outgoingTelemetry.battery_v = 11.8; // Simulated or measured via voltage divider pin
    outgoingTelemetry.throttle = map(incomingReadings.joystick2_Y, 0, 4095, 0, 100);
    outgoingTelemetry.gate_pos = incomingReadings.payloadRelease ? 100 : 0;
    outgoingTelemetry.payload_mass = currentPayloadMass;
    outgoingTelemetry.rssi = -65; // Optional Wi-Fi RSSI retrieval if needed
    outgoingTelemetry.packet_loss = 0;
    outgoingTelemetry.frame_count = frameCounter;

    // Send Telemetry back to Ground Station via ESP-NOW
    esp_now_send(groundStationAddress, (uint8_t *) &outgoingTelemetry, sizeof(outgoingTelemetry));
  }
}