# UWAC-BP26-Firmware

## Overview

Firmware for the UWAC Beginners' Rocketry 2026 flight computer. Uses non-blocking state machine design.

## Helper Functions
1. `transitionTo(FlightState newState)` Change to current state and print to serial.
- newState: ENUM State that the currState will transition to
2. `firePyroLength(int pin, unsigned long &startTimeRef, bool &firedFlag)`
Tracks time manually rather than using delay function.
- pin: Physical hardware pin connected to pyro
- &startTimeRef: Points to global timer variables
- &firedFlag: Boolean to check for deployment

## Technical Details

- **Delta-Time Calculation**: Time that passes between loop
- **EMA Sensor Filter**: EMA to smooth out sensor noise spikes
- **Continuity Checks**: Prevent premature ignition of pyros

## To-Do (depends on hardware)
- [ ] Buzzer / Flashing LEDs Code
- [ ] External SD Card Logging
