#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <TinyGPS++.h>

// MPU6050 and Ultrasonic Constants
Adafruit_MPU6050 mpu;
const int trigPin1 = 5;  // Ultrasonic Sensor 1 Trigger Pin
const int echoPin1 = 18; // Ultrasonic Sensor 1 Echo Pin
const int trigPin2 = 22; // Ultrasonic Sensor 2 Trigger Pin
const int echoPin2 = 23; // Ultrasonic Sensor 2 Echo Pin
const int ledPin = 4;    // LED Pin
const int buzzerPin = 15; // Buzzer Pin

#define SOUND_SPEED 0.034 // cm/us -> Speed of sound
#define I2C_SDA 21
#define I2C_SCL 19

// GPS Pins
#define RX2 25 // GPS Module RX pin
#define TX2 26 // GPS Module TX pin
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

// WiFi and ThingSpeak Credentials
const char* ssid = "Sesuuu";
const char* password = "nnnnnnnn";
WiFiClient client;
unsigned long myChannelNumber = 2780486;
const char* myWriteAPIKey = "8IVHBFWZ7FF26I9Q";

// Variables
long duration1, duration2;
float distanceCm1, distanceM1, distanceCm2;
float acceleration, relativeVelocity;
double latitude = 0.0, longitude = 0.0;
unsigned long t = 1; // Time interval in seconds

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SDA, I2C_SCL);
  SerialGPS.begin(9600, SERIAL_8N1, RX2, TX2);

  // MPU6050 Initialization
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // WiFi Initialization
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi.");
}

float readUltrasonicSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * SOUND_SPEED / 2; // Distance in cm
}

void readGPSData() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      } else {
        Serial.println("GPS location is invalid.");
      }
    }
  }
}

void loop() {
  // Read Ultrasonic Sensor Data
  distanceCm1 = readUltrasonicSensor(trigPin1, echoPin1); // US1 Distance (cm)
  distanceM1 = distanceCm1 / 100.0;                      // Convert US1 Distance to meters
  distanceCm2 = readUltrasonicSensor(trigPin2, echoPin2); // US2 Distance (cm)

  // Read MPU6050 Sensor Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  acceleration = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2)) - 9.81;
  relativeVelocity = fabs((distanceM1 / t) - (acceleration * t)); // Relative velocity (m/s)

  // LED and Buzzer Logic
  if ((distanceCm1 > 0 && distanceCm1 <= 10) || (distanceCm2 > 0 && distanceCm2 <= 20)) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  if (relativeVelocity > 5) {
    digitalWrite(buzzerPin, HIGH);
    Serial.println("Forward Collision Detected! Buzzer Activated.");
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  // Read GPS Data
  readGPSData();

  // Print Data to Serial Monitor
  Serial.print("US1 Distance (m): ");
  Serial.println(distanceM1);
  Serial.print("US2 Distance (cm): ");
  Serial.println(distanceCm2);
  Serial.print("Acceleration (m/s^2): ");
  Serial.println(acceleration);
  Serial.print("Temperature (C): ");
  Serial.println(temp.temperature);
  Serial.print("Relative Velocity (m/s): ");
  Serial.println(relativeVelocity);
  Serial.print("Latitude: ");
  Serial.println(latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);

  // Send Data to ThingSpeak
  ThingSpeak.setField(1, distanceM1);
  ThingSpeak.setField(2, distanceCm2);
  ThingSpeak.setField(3, acceleration);
  ThingSpeak.setField(4, temp.temperature);
  ThingSpeak.setField(5, relativeVelocity);
  ThingSpeak.setField(6, (float)latitude);
  ThingSpeak.setField(7, (float)longitude);

  int status = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (status == 200) {
    Serial.println("Data successfully sent to ThingSpeak.");
  } else {
    Serial.println("Error sending data to ThingSpeak: " + String(status));
  }

  delay(1000); // 1-second delay
}
