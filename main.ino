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

#define DROGUE_PYRO   PB8
#define MAIN_PYRO     PB9

enum FlightState{
  s_IDLE,
  s_BOOST,
  s_COAST,
  s_APOGEE,
  s_DESCENT,
  s_LANDED
}

// Deployment & launch settings
float launchThresh    2;      // 2 G-force
float mainDeployAlt   300.0;  // Deploy main chute at 300 metres (AGL)
float firingDuration  1000.0; // Firing duration at 1000 milliseconds

// Global tracking variables
float groundAlt = 0.0;
float currAlt = 0.0;
float maxAlt = 0.0;
float currVertAccel = 0.0;
float currVertVelocity = 0.0;
float lastAlt = 0.0;
unsigned long lastUpdateMs = 0;

// Pyro timing
unsigned long drogueFireStartMs = 0;
bool drogueFired = false;
unsigned long mainFireStartMs = 0;
bool mainFired = false;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:


}
