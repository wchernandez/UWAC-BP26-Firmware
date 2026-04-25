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

// Custom serial object
HardwareSerial Serial1(PA10, PA9);
#define Serial Serial1

// Pyro Channels
#define CH1 PB8 // Drogue
#define CH2 PB9 // Main

#define SDA PB7
#define SCL PB6

// Continuity Channels (Not Allocated)
#define CH1_ADC PB14 // Drogue
#define CH2_ADC PB15 // Main
 
// Status LED & Buzzer
#define LED_PIN PC13
#define BUZZ_PIN PB10

// LED & Buzzer States
#define LED_ON HIGH
#define LED_OFF LOW
#define BUZZ_ON HIGH
#define BUZZ_OFF LOW
#define BUZZ_FREQ 2000 // Tone frequency in Hz (passive buzzers)

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
const float LAUNCH_THRESH = 2.0; // 2 G-force
const float MAIN_DEPLOY_ALT = 300.0; // Deploy main chute at 300 metres (AGL)
const float FIRING_DURATION = 1000.0; // Firing duration at 1000 milliseconds
const float GRAVITY_ACCEL = 9.81; // m/s^2 for conversions
const float FAILSAFE_VELOCITY = -25.0; // Failsafe if falling > 25 m/s (drogue failure)

// Global tracking variables
float groundAlt = 0.0;
float currAlt = 0.0;
float maxAlt = 0.0;
float currAccel = 0.0;
float currZVel = 0.0;
float lastAlt = 0.0;
unsigned long lastUpdateMs = 0;
unsigned long apogeeDetectMs = 0; // Debounce tracker for apogee

// Pyro timing
unsigned long drogueFireStartMs = 0;
bool drogueFired = false;
unsigned long mainFireStartMs = 0;
bool mainFired = false;

// Diagnostic Tracking
int diagCategory = 0;
int diagCode = 0;

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
    } else if (millis() - startTimeRef > FIRING_DURATION) {
      digitalWrite(pin, LOW); // Cut off
      firedFlag = true;
      Serial.print("PYRO FIRING COMPLETE ON PIN: "); Serial.println(pin);
    }
  }
}

// LED blink code for diagnostics
void blinkCode(int category, int code) {
  // Clear LED state
  digitalWrite(LED_PIN, LED_OFF);
  digitalWrite(BUZZ_PIN, BUZZ_OFF);
  delay(1000);

  // Blink category (long blinks)
  for (int i = 0; i < category; i++) {
    digitalWrite(LED_PIN, LED_ON);
    tone(BUZZ_PIN, BUZZ_FREQ, 500);
    delay(500);
    digitalWrite(LED_PIN, LED_OFF);
    delay(500);
  }

  delay(1000); // Seperate category and code

  // Blink code (short blinks)
  for (int i = 0; i < code; i++) {
    digitalWrite(LED_PIN, LED_ON);
    tone(BUZZ_PIN, BUZZ_FREQ, 200);
    delay(200);
    digitalWrite(LED_PIN, LED_OFF);
    delay(200);
  }

  delay(2000); // Separate next sequence
}

// Flash/Buzz the max altitude starting from lowest significant digit
void blinkAltitude(int alt) {
  if (alt <= 0) return;
  
  int tempAlt = alt;
  while (tempAlt > 0) {
    int digit = tempAlt % 10;
    tempAlt /= 10;

    if (digit == 0) {
      digitalWrite(LED_PIN, LED_ON);
      tone(BUZZ_PIN, BUZZ_FREQ, 250);
      delay(1000); // Long flash for 0
      digitalWrite(LED_PIN, LED_OFF);
    } else {
      for (int i = 0; i < digit; i++) {
        digitalWrite(LED_PIN, LED_ON);
        tone(BUZZ_PIN, BUZZ_FREQ, 250);
        delay(250);
        digitalWrite(LED_PIN, LED_OFF);
        delay(250);
      }
    }
    delay(1000); // Pause between digits
  }
  delay(3000); // Long pause before repeating
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CH1, OUTPUT);
  pinMode(CH2, OUTPUT);
  digitalWrite(CH1, 0);
  digitalWrite(CH2, 0);

  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin();

  // Initialise Status LED and Buzzer
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, BUZZ_OFF);
  
  // Simple startup tone
  tone(BUZZ_PIN, BUZZ_FREQ, 500); 
  delay(500);

  // Initialise MPU6050 (IMU)
  if (!imu.begin()) {
    Serial.println("Cannot detect MPU6050.");
    diagCategory = 2; diagCode = 1;
    while (1) {
      blinkCode(diagCategory, diagCode);
    }
  } else {
    imu.setAccelerometerRange(MPU6050_RANGE_16_G);
  }

  // Initalise BMP280
  if (!baro.begin(0x76)) {
    Serial.println("Cannot detect BMP280.");
    diagCategory = 3; diagCode = 1;
    while (1) {
      blinkCode(diagCategory, diagCode);
    }
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
    diagCategory = 4; // Category 4: Pyro

    if (analogRead(CH1_ADC) < 100 && analogRead(CH2_ADC) < 100) {
      Serial.println("WARNING: BOTH Pyros disconnected!");
      diagCode = 3;
    } else if (analogRead(CH1_ADC) < 100) {
      Serial.println("WARNING: CH1 (Drogue) Pyro disconnected!");
      diagCode = 1;
    } else if (analogRead(CH2_ADC) < 100) {
      Serial.println("WARNING: CH2 (Main) Pyro disconnected!");
      diagCode = 2;
    }

    blinkCode(diagCategory, diagCode);
  }

  // Signal Success
  diagCategory = 1; diagCode = 1;
  blinkCode(diagCategory, diagCode);

  Serial.println("Continuity checks passed. All pyros armed.");
  Serial.println("Flight Computer Initialised. State: IDLE");

  lastUpdateMs = millis(); // Reset loop timer to discard setup delays
}

void loop() {
  unsigned long currMs = millis();

  // Enforce a 50Hz (20ms) loop heartbeat to prevent division by zero and stabilise dt calculations for velocity derivation.
  if (currMs - lastUpdateMs < 20) {
    return;
  }

  float dt = (currMs - lastUpdateMs) / 1000.0; // Seconds elapsed
  lastUpdateMs = currMs;

  // Reading sensors
  sensors_event_t a, g, temp;
  if(imu.getEvent(&a, &g, &temp)){
    currAccel = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  }

  static bool firstLoop = true;
  
  float rawAlt = baro.readAltitude(1013.25);
  static float filteredAlt = -999.0;
  if (firstLoop) {
    filteredAlt = rawAlt; // Initialise on first loop
  }
  
  // EMA Filter to smooth out sensor noise spikes
  filteredAlt = (filteredAlt * 0.85) + (rawAlt * 0.15);
  currAlt = filteredAlt - groundAlt;

  // Secondary EMA filter to prevent erroneous failsafe triggers.
  float rawZVel = (currAlt - lastAlt) / dt;
  static float filteredZVel = 0.0;
  if (firstLoop) {
    filteredZVel = rawZVel; // Initialise to first velocity 
    firstLoop = false;
  }
  filteredZVel = (filteredZVel * 0.80) + (rawZVel * 0.20);
  currZVel = filteredZVel;

  lastAlt = currAlt;

  if (currAlt > maxAlt) {
    maxAlt = currAlt;
  }

  // State machine logic
  static unsigned long landedCheckMs = currMs; // For landing, debounce
  
  switch(currState) {
    case s_IDLE:
      // Check if acceleration in G's exceeds threshold while altitude is increasing
      if ((currAccel / GRAVITY_ACCEL) > LAUNCH_THRESH && currAlt > 2.0){
        transitionTo(s_BOOST);
      }
      break;

    case s_BOOST:
      // Motor burnout: Accel drops significantly - adjusted to account for drag
      if (currAccel < 13.0) { 
        transitionTo(s_COAST);
      }
      break;

    case s_COAST:
      // Detect apogee when altitude drops 2m from maximum (debounced to avoid false trigger)
      if ((maxAlt - currAlt) > 2.0) { 

        // Check if debounce timer is zero, if it is, set it to current time. Otherwise check if debounce timer has been met
        if (apogeeDetectMs == 0){
          apogeeDetectMs = currMs;
        } else if (currMs - apogeeDetectMs >= 200) {
          transitionTo(s_APOGEE);
        }
      } else {
        apogeeDetectMs = 0; // Reset debouncer if altitude fluctuates back up
      }
      break;

    case s_APOGEE:
      firePyroLength(CH1, drogueFireStartMs, drogueFired);
      if (drogueFired) {
        transitionTo(s_DESCENT);
      }
      break;

    case s_DESCENT:
      // Deploy main if altitude reached OR if failsafe is triggered (falling too fast)
      if (currAlt <= MAIN_DEPLOY_ALT || currZVel <= FAILSAFE_VELOCITY) {
        firePyroLength(CH2, mainFireStartMs, mainFired);
      }

      // Checks for low velocity to detect landing
      if (abs(currZVel) < 1.5) {
        if (currMs - landedCheckMs > 5000) {
          transitionTo(s_LANDED);
        }
      } else {
        landedCheckMs = currMs; 
      }
      break;

    case s_LANDED:
      // Ensure pyros are turned off in case they were still firing during landing (safety)
      digitalWrite(CH1, LOW);
      digitalWrite(CH2, LOW);
      
      // Play back the maximum altitude achieved
      blinkAltitude((int)maxAlt);
      break;
  }
}