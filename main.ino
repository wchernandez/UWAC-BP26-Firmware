/*
  UWAC Beginner's Rocketry 2026 Firmware
*/

// Standard Libraries
#include <Arduino.h>
#include <Wire.h>

// Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>

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
FlightState currState = s_IDLE;

// Sensor Instances
Adafruit_MPU6050 imu;
Adafruit_BMP280 baro;

// Deployment & launch settings
float launchThresh = 2.0; // 2 G-force
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

// Helper function
void transitionTo(FlightState newState){
  Serial.print("Transitioning to State: ");
  Serial.println(newState);
  currState = newState;
}
// Non-blocking function to fire pyros over specific length
void firePyroLength(int pin, unsigned long &startTimeRef, bool &firedFlag) {
  if (!firedFlag) {
    if (startTimeRef == 0) {
      startTimeRef = millis();
      digitalWrite(pin, HIGH); // Ignite
      Serial.print("FIRING PYRO ON PIN: "); Serial.println(pin);
    } else if (millis() - startTimeRef > firingDuration) {
      digitalWrite(pin, LOW); // Cut off
      firedFlag = true;
      Serial.print("PYRO FIRING COMPLETE ON PIN: "); Serial.println(pin);
    }
  }
}

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
  unsigned long currMs = millis();
  float dt = (currMs - lastUpdateMs) / 1000.0; // Seconds elapsed

  if (dt <= 0.0) return; // Waits until time elapses

  lastUpdateMs = currMs;

  // Reading sensors
  sensors_event_t a, g, temp;
  if(imu.getEvent(&a, &g, &temp)){
    currZAccel = a.acceleration.z;
  }

  float rawAlt = baro.readAltitude(1013.25);
  static float filteredAlt = -999.0;
  if (filteredAlt == -999.0) filteredAlt = rawAlt; // Initialise on first loop

  // EMA Filter to smooth out sensor noise spikes
  filteredAlt = (filteredAlt * 0.85) + (rawAlt * 0.15);
  currAlt = filteredAlt - groundAlt;
  currZVel = (currAlt - lastAlt) / dt;
  lastAlt = currAlt;

  if (currAlt > maxAlt) {
    maxAlt = currAlt;
  }

  // State machine logic
  static unsigned long landedCheckMs = currMs; // For landing, debounce
  
  switch(currState) {
    case s_IDLE:
      if (abs(currZAccel) > launchThresh && currAlt > 2.0){
        transitionTo(s_BOOST);
      }
      break;

    case s_BOOST:
      if (abs(currZAccel) < 5.0) { // Motor burnout
        transitionTo(s_COAST);
      }
      break;

    case s_COAST:
      // Detect apogee when altitude drops 2m from maximum
      if ((maxAlt - currAlt) > 2.0) { 
        transitionTo(s_APOGEE);
      }
      break;

    case s_APOGEE:
      firePyroLength(CH1, drogueFireStartMs, drogueFired);
      if (drogueFired) {
        transitionTo(s_DESCENT);
      }
      break;

    case s_DESCENT:
      if (currAlt <= mainDeployAlt) {
        firePyroLength(CH2, mainFireStartMs, mainFired);
      }

      // Checks for main chute fired, low velocity and near ground
      if (mainFired && abs(currZVel) < 0.5 && currAlt < 15.0) {
        if (currMs - landedCheckMs > 5000) {
          transitionTo(s_LANDED);
        }
      } else {
        landedCheckMs = currMs; 
      }

    case s_LANDED:
      // Standby
      break;
  }
}
