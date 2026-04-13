/*
UWAC Beginner's Rocketry 2026
*/

// Standard Libraries
#include <Arduino.h>
#include <Wire.h>

// Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>

// // Data Logging and Storage Libraries
// #include <SdFat.h>

// Pyro Channels
#define CH1 PB8 // Drogue
#define CH2 PB9 // Main

// Continuity Channels (Not Allocated)
#define CH1_ADC 0 // Drogue
#define CH2_ADC 0 // Main

// Flight States
enum FlightState {
  s_IDLE,
  s_BOOST,
  s_COAST,
  s_APOGEE,
  s_DESCENT,
  s_LANDED
};

// Set initial state to idle
FlightState currentState = s_IDLE;

// Sensor Instances
Adafruit_MPU6050 imu;
Adafruit_BMP280 baro;

// Deployment & launch settings
float launchThresh = 2; // 2 G-force
float mainDeployAlt = 300.0; // Deploy main chute at 300 metres (AGL)
float firingDuration = 1000.0; // Firing duration at 1000 milliseconds

// Global tracking variables
float groundAlt = 0.0;
float currAlt = 0.0;
float maxAlt = 0.0;
float currZAccel = 0.0;
float currZVel = 0.0;
float lastAlt = 0.0;
unsigned long lastUpdateMs = 0;

// Pyro timing
unsigned long drogueFireStartMs = 0;
bool drogueFired = false;
unsigned long mainFireStartMs = 0;
bool mainFired = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CH1, OUTPUT);
  pinMode(CH2, OUTPUT);
  digitalWrite(CH1, 0);
  digitalWrite(CH2, 0);

  Wire.begin();

  // Initialise MPU6050 (IMU)
  if (!imu.begin()) {
    Serial.println("Cannot detect MPU6050.");
  } else {
    imu.setAccelerometerRange(MPU6050_RANGE_16_G);
  }

  // Initalise BMP280
  if (!baro.begin()) {
    Serial.println("Cannot detect BMP280.");
  } else {
    // Average ground pressure (1013.25Pa) at sea level over 1 second
    for (int i=0; i<10; i++) { 
      groundAlt += baro.readAltitude(1013.25); 
      delay(50);
    }
    groundAlt /= 10.0;
  }

  // Pre-flight Checks

  // ADC Channel Check
  pinMode(CH1_ADC, INPUT);
  pinMode(CH2_ADC, INPUT);
  while (analogRead(CH1_ADC) < 100 || analogRead(CH2_ADC) < 100) {
    if (analogRead(CH1_ADC) < 100) Serial.println("WARNING: CH1 (Drogue) Pyro disconnected!");
    if (analogRead(CH2_ADC) < 100) Serial.println("WARNING: CH2 (Main) Pyro disconnected!");
    delay(1000);
  }

  Serial.println("Continuity checks passed. All pyros armed.");
  Serial.println("Flight Computer Initialised. State: IDLE");
}

void loop() {
  void unsigned currentMs = millis()
  

}
