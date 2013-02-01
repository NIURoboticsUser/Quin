
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

#define MOTOR_LEFT_PIN 10
#define MOTOR_BACK_PIN 6
#define MOTOR_FRONT_PIN 3
//#define MOTOR_LEFT_RAW_OFFSET 19
//#define MOTOR_LEFT_THRUST_OFFS

// Motor Controller
#include "MotorController.h"
MotorController motorController;


// PID Controller
#include "PID.h"
PID pitchPid(PID_PITCH_P, PID_PITCH_I, PID_PITCH_D, PID_PITCH_IMAX);
PID yawPid(PID_YAW_P, PID_YAW_I, PID_YAW_D, PID_YAW_IMAX);
PID rollPid(PID_ROLL_P, PID_ROLL_I, PID_ROLL_D, PID_ROLL_IMAX);
PID altitudePid(PID_ALT_P, PID_ALT_I, PID_ALT_D, PID_ALT_IMAX);

boolean kill, noDof;
int targetThrust = 0;
unsigned long lastCollectionTime;

int32_t lastWx = 0, lastWy = 0, lastWz = 0;

void setup() {
  Serial.begin(38400);
  
  
  dofHandler.begin(9600, 28800);
  dofHandler.setDataMode(DOF_DATA_MODE_EULER);
  dofHandler.setUpdateInterval(30);
  dofHandler.requestData();
  
#ifdef ARM_ON_LOAD
  delay(5000);
  motorController.armMotors();
  delay(1000);
#endif
}

void loop() {
  if(Serial.available()) {
    boolean setThrust = false;
    char c = Serial.peek();
    if (c == 'z') {
      Serial.read();
      dofHandler.zeroCalibrate();
    } else if (c == 'k') {
      Serial.read();
      kill = !kill;
      noDof = kill;
      motorController.setMotorSpeed(MOTOR_ALL, 0);
    } else if (c == 's') {
      Serial.read();
      noDof = true;
      Serial.println("Dropping");
      for (int i = 0; i < 20; i++) {
        motorController.subtractMotorSpeed(MOTOR_ALL, 13);
        delay(100);
      }
      Serial.println("Dropped");
      noDof = true;
    } else if (c == 'r') {
      Serial.read();
      dofHandler.zeroCalibrate();
      kill = false;
      noDof = false;
      
      pitchPid.resetI();
      yawPid.resetI();
      rollPid.resetI();
      altitudePid.resetI();
    } else if (c == 't') {
      Serial.read();
      setThrust = true;
      delay(10);
    } else if (c == 'p') {
      Serial.read();
      delay(10);
    }
 
    static char charray[10];
    memset(charray, 0, 9);
   
    for(int i=0; i<10 && Serial.available(); i++){
      delay(5);
      charray[i]=Serial.read();
    }
    
    int num = atoi(charray);
    
    //for(int i=MOTOR_FRONT; i<=MOTOR_BACK; i++) motorValues[i]+=num;
    if (c == 't') {
      motorController.setMotorThrust(MOTOR_ALL, num);
      Serial.print("Thrust: "); Serial.println(num);
    } else if (c == 'p') {
      pitchPid.setkP(num / 100.0);
      rollPid.setkP(num / 100.0);
    } else {
      //controller.setMotorSpeed(MOTOR_ALL, num);
      targetThrust = num;
      //motorController.setMotorRaw(MOTOR_ALL, num);
      //motorController.setMotorRaw(MOTOR_LEFT, num + MOTOR_LEFT_RAW_OFFSET);
      //motorController.setMotorRaw(MOTOR_RIGHT, num + MOTOR_RIGHT_RAW_OFFSET);
      //motorController.setMotorRaw(MOTOR_FRONT, num + MOTOR_FRONT_RAW_OFFSET);
      //motorController.setMotorRaw(MOTOR_BACK, num + MOTOR_BACK_RAW_OFFSET);
    }
    //targetSpeed = constrain(targetSpeed + num, 0, 255);
    
  }
  //Serial.println(motorController.getMotorRaw(MOTOR_FRONT));
  
  dofHandler.checkStreamValid();
  if (true && !noDof && dofHandler.isNewDataAvailable(true)) {
    unsigned long now = millis();
    dofHandler.requestData();
    double dt = (now - lastCollectionTime) / 1000.0;
    lastCollectionTime = now;
    EulerData eulerData = dofHandler.getEulerData();
    int32_t wx = (int32_t)(-eulerData.pitch * RAD_TO_DEG * 10);
    int32_t wy = (int32_t)(-eulerData.roll * RAD_TO_DEG * 10);
    int32_t wz = (int32_t)(eulerData.yaw * RAD_TO_DEG * 10);
    
    int32_t commandVars[4] = {0}; // Pitch, roll, yaw, altitude
    
    commandVars[0] = pitchPid.getPid(wx, now);
    commandVars[1] = rollPid.getPid(wy, now);
    commandVars[2] = yawPid.getPid(wz, now);
    
    double thrusts[4] = {targetThrust, targetThrust, targetThrust, targetThrust};
    
    double wox = (wx - lastWx) / dt / 10.0;
    double woy = (wy - lastWy) / dt / 10.0;
    double woz = (wz - lastWz) / dt / 10.0;
    
    double Tx = (-wox + sqrt(wox*wox + 2 * (commandVars[0] / 10.0) * PID_STABLE_SCALAR)) / (PID_STABLE_SCALAR);
    double Ty = (-woy + sqrt(woy*woy + 2 * (commandVars[1] / 10.0) * PID_STABLE_SCALAR)) / (PID_STABLE_SCALAR);
    double Tz = (-wox + sqrt(woz*woz + 2 * (commandVars[2] / 10.0) * PID_STABLE_SCALAR)) / (PID_STABLE_SCALAR);
    
    // Pitch
    thrusts[0] -= ((commandVars[0] / 10.0) * PID_PITCH_SCALAR);
    thrusts[2] += ((commandVars[0] / 10.0) * PID_PITCH_SCALAR);
    
    // Roll
    thrusts[1] += ((commandVars[1] / 10.0) * PID_ROLL_SCALAR);
    thrusts[3] -= ((commandVars[1] / 10.0) * PID_ROLL_SCALAR);
    
    // Yaw
    //thrusts[0] += (Tz * PID_YAW_SCALAR);
    //thrusts[2] += (Tz * PID_YAW_SCALAR);
    //thrusts[1] -= (Tz * PID_YAW_SCALAR);
    //thrusts[3] -= (Tz * PID_YAW_SCALAR);
    
    motorController.setMotorThrust(MOTOR_FRONT, constrain(thrusts[0], 0, 65535));
    motorController.setMotorThrust(MOTOR_RIGHT, constrain(thrusts[1], 0, 65535));
    motorController.setMotorThrust(MOTOR_BACK, constrain(thrusts[2], 0, 65535));
    motorController.setMotorThrust(MOTOR_LEFT, constrain(thrusts[3], 0, 65535));
    
    

    Serial.print("Pitch: "); Serial.print(eulerData.pitch, 3);
    Serial.print(", Roll: "); Serial.print(eulerData.roll, 3);
    Serial.print(", Yaw: "); Serial.print(eulerData.yaw, 3);
    Serial.print(", thrusts[0]: "); Serial.print(thrusts[0]);
    Serial.print(", thrusts[1]: "); Serial.print(thrusts[1]);
    Serial.print(", thrusts[2]: "); Serial.print(thrusts[2]);
    Serial.print(", thrusts[3]: "); Serial.print(thrusts[3]);
    Serial.println();
    lastWx = wx;
    lastWy = wy;
    lastWz = wz;
  }
  
}


