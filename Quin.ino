
#include "Arduino.h"
#include "utility.h"
#include "Config.h"

// Dof Handler
#include "DofHandler.h"
#ifndef ARDUINO_MEGA
  #include <SoftwareSerial.h>
  SoftwareSerial dofSerial(2, 3); // RX, TX
  DofHandler<SoftwareSerial> dofHandler(&dofSerial);
#else
  DofHandler<HardwareSerial> dofHandler(&Serial1);
#endif

// Motor Controller
#include "MotorController.h"
MotorController motorController;


// PID Controller
#include "PID.h"
PID pitchPid, yawPid, rollPid, altitudePid;


void setup() {
  
}

void loop() {
  
}


