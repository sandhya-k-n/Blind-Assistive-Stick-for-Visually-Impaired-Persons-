/*
  ==========================================================================
  BLIND ASSISTIVE STICK FOR VISUALLY IMPAIRED PERSONS
  ==========================================================================
  Board      : ESP32 Dev Module
  Sensors    :
    - HC-SR04   : Ultrasonic obstacle detection
    - MPU6050   : Fall detection (accelerometer + gyroscope, I2C)
    - SIM800L   : GSM module for SMS alerts (UART)
    - NEO-6M    : GPS module for real-time location (UART)
  Output     : Buzzer + Vibration motor for obstacle alerts
  Power      : Li-ion battery (via TP4056/buck converter to 3.3V/5V rails)

  Libraries required (install via Library Manager):
    - Adafruit MPU6050
    - Adafruit Unified Sensor
    - TinyGPSPlus
  ==========================================================================
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// ------------------- PIN DEFINITIONS -------------------
// HC-SR04 Ultrasonic Sensor
#define TRIG_PIN      5
#define ECHO_PIN      18

// Alert outputs
#define BUZZER_PIN    19
#define VIBRATION_PIN 23

// SIM800L (UART2)
#define SIM800_RX     16   // ESP32 RX2 <- SIM800 TX
#define SIM800_TX     17   // ESP32 TX2 -> SIM800 RX

// NEO-6M GPS (UART1)
#define GPS_RX        26   // ESP32 RX1 <- GPS TX
#define GPS_TX        27   // ESP32 TX1 -> GPS RX

// ------------------- THRESHOLDS -------------------
#define OBSTACLE_DISTANCE_CM   60      // Alert if obstacle closer than this
#define FALL_THRESHOLD_G       2.5     // Acceleration magnitude threshold (in g) for fall
#define FALL_CONFIRM_DELAY_MS  1500    // Time to confirm fall (check for stillness after)
#define GUARDIAN_NUMBER  "+91XXXXXXXXXX"   // Replace with guardian's phone number

// ------------------- GLOBAL OBJECTS -------------------
Adafruit_MPU6050 mpu;
HardwareSerial sim800(2);   // UART2 for SIM800L
HardwareSerial gpsSerial(1); // UART1 for NEO-6M
TinyGPSPlus gps;

unsigned long lastObstacleCheck = 0;
unsigned long lastFallCheck = 0;
unsigned long lastSMSTime = 0;
const unsigned long SMS_COOLDOWN = 60000; // Prevent SMS spam (1 min cooldown)

bool fallAlertSent = false;

// ==========================================================================
// SETUP
// ==========================================================================
void setup() {
  Serial.begin(115200);

  // Obstacle sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(VIBRATION_PIN, LOW);

  // I2C for MPU6050
  Wire.begin();  // default SDA=21? Note: pin 21 is used for vibration above.
                 // If using default ESP32 I2C pins (21,22), reassign vibration pin.
                 // Wire.begin(customSDA, customSCL) can be used instead, e.g. Wire.begin(4,22);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected! Check wiring.");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 initialized.");
  }

  // SIM800L UART init
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(3000); // Allow SIM800L to register on network
  initSIM800();

  // GPS UART init
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("System Ready: Blind Assistive Stick");
}

// ==========================================================================
// MAIN LOOP
// ==========================================================================
void loop() {
  unsigned long now = millis();

  // 1. Obstacle detection (runs frequently)
  if (now - lastObstacleCheck >= 150) {
    lastObstacleCheck = now;
    checkObstacle();
  }

  // 2. Fall detection
  if (now - lastFallCheck >= 200) {
    lastFallCheck = now;
    checkFall();
  }

  // 3. Continuously feed GPS data to parser
  readGPS();
}

// ==========================================================================
// OBSTACLE DETECTION (HC-SR04)
// ==========================================================================
long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m range)
  if (duration == 0) return -1; // No echo received (out of range)

  long distance = duration * 0.0343 / 2; // Speed of sound = 343 m/s
  return distance;
}

void checkObstacle() {
  long distance = getDistanceCM();

  if (distance > 0 && distance <= OBSTACLE_DISTANCE_CM) {
    // Obstacle detected - alert user
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(VIBRATION_PIN, HIGH);

    // Optional: vary alert intensity/frequency based on proximity
    if (distance <= 20) {
      // Very close - rapid beep pattern handled by short buzzer bursts
      Serial.print("DANGER! Obstacle very close: ");
    } else {
      Serial.print("Obstacle detected: ");
    }
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(VIBRATION_PIN, LOW);
  }
}

// ==========================================================================
// FALL DETECTION (MPU6050)
// ==========================================================================
void checkFall() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate total acceleration magnitude (in g, 1g = 9.8 m/s^2)
  float ax = a.acceleration.x / 9.8;
  float ay = a.acceleration.y / 9.8;
  float az = a.acceleration.z / 9.8;
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);

  if (accelMagnitude >= FALL_THRESHOLD_G && !fallAlertSent) {
    Serial.println("Sudden impact detected! Confirming fall...");
    delay(FALL_CONFIRM_DELAY_MS);

    // Re-check orientation/stillness to confirm a fall (reduces false positives)
    sensors_event_t a2, g2, temp2;
    mpu.getEvent(&a2, &g2, &temp2);
    float gyroMag = sqrt(g2.gyro.x * g2.gyro.x +
                          g2.gyro.y * g2.gyro.y +
                          g2.gyro.z * g2.gyro.z);

    if (gyroMag < 0.5) { // Low rotation = person likely stationary/fallen
      Serial.println("FALL CONFIRMED! Sending alert...");
      fallAlertSent = true;
      sendFallAlert();
    }
  }

  // Reset fall flag after some time so future falls can be detected again
  if (fallAlertSent && millis() - lastSMSTime > SMS_COOLDOWN) {
    fallAlertSent = false;
  }
}

// ==========================================================================
// GPS (NEO-6M via TinyGPSPlus)
// ==========================================================================
void readGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

String getLocationString() {
  if (gps.location.isValid()) {
    String lat = String(gps.location.lat(), 6);
    String lng = String(gps.location.lng(), 6);
    String mapsLink = "https://maps.google.com/?q=" + lat + "," + lng;
    return "Lat: " + lat + ", Lng: " + lng + " | " + mapsLink;
  } else {
    return "Location unavailable (no GPS fix yet)";
  }
}

// ==========================================================================
// SIM800L GSM - INITIALIZATION & SMS
// ==========================================================================
void initSIM800() {
  sendATCommand("AT", 1000);            // Check module response
  sendATCommand("AT+CMGF=1", 1000);     // Set SMS to text mode
  sendATCommand("AT+CNMI=1,2,0,0,0", 1000); // Route incoming SMS to serial (optional)
}

String sendATCommand(String command, int waitTime) {
  sim800.println(command);
  long t = millis();
  String response = "";
  while (millis() - t < waitTime) {
    while (sim800.available()) {
      response += (char)sim800.read();
    }
  }
  Serial.println(response);
  return response;
}

void sendFallAlert() {
  if (millis() - lastSMSTime < SMS_COOLDOWN) return; // Avoid spamming

  String location = getLocationString();
  String message = "ALERT: Fall detected!\n" + location;

  sim800.println("AT+CMGF=1");              // Text mode
  delay(200);
  sim800.print("AT+CMGS=\"");
  sim800.print(GUARDIAN_NUMBER);
  sim800.println("\"");
  delay(200);
  sim800.print(message);
  delay(200);
  sim800.write(26);  // CTRL+Z to send SMS
  delay(3000);

  lastSMSTime = millis();
  Serial.println("Fall alert SMS sent: " + message);
}
