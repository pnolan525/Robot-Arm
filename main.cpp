#include <Adafruit_PWMServoDriver.h>    
#include <N64Controller.h>
#include <Tic.h>        //BLACK-2B BLUE-2A GRN-1A RED-1B
#include <Wire.h>       //SDA-A4  - SCL-A5
#include <Arduino.h>

// I2C Objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
TicI2C tic(14);

// Servo Variables
const uint8_t shoulder1 = 0;
const uint8_t shoulder2 = 1;
const uint8_t elbow = 2;
const uint8_t wrist1 = 3;
const uint8_t wrist2 = 4;
const uint8_t claw = 5;
const uint8_t numServos = 6;

// Servo Position Arrays
float homePos[numServos] = {810, 2210, 2500, 1500, 1500, 510};            //array of initial servo positions
float smoothPos[numServos] = {810, 2210, 2500, 1500, 1500, 510};          //array of current servo positions
float targetPos[numServos] = {810, 2210, 2500, 1500, 1500, 510};          //array of target servo positions
const float servoMin[numServos] = {810, 850, 550, 550, 550, 510};         // Min pulse length for 0°
const float servoMax[numServos] = {2150, 2210, 2500, 2450, 2450, 2450};   // Max pulse length for 180°

// Smoothing factor (0.0–1.0, lower = smoother)
const float alpha[numServos] = {0.05, 0.05, 0.05, 1.0, 1.0, 1.0};         // Smoothing factor
const float servoInc[numServos] = {10, 10, 10, 15, 15, 15};               // Servo Increments ms

// Stepper Variables
uint8_t posStepper = 0;       // base position variable
const uint8_t stepInc = 10;   // Stepper Increments

//Arduino Pins
N64Controller Ncon(13);       // N64 Controller pin

//Set a new target position
void setTarget(int servo, int pulse) {
  targetPos[servo] = constrain(pulse, servoMin[servo], servoMax[servo]);
}
//Retrun Robot Arm to home position
void home(){
  for (uint8_t i = 0; i < numServos; i++) {
    setTarget(i, homePos[i]);
  }
}

void setup() {
  //Initialize communications
  
  //Serial communication for debugging
  //Serial.begin(9600);  

  //Tic for stepper motor control   
  tic.setProduct(TicProduct::T249);
  Wire.begin();
  tic.exitSafeStart();
  tic.setCurrentLimit(1000); // mA
  posStepper = tic.getCurrentPosition();
  tic.haltAndSetPosition(posStepper);
  //PCA9685 for servo control
  pwm.begin();                            
  pwm.setOscillatorFrequency(23800000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);
  //N64 Controller communication
  Ncon.begin();             
  delay(10);
  
  for(uint8_t i = 0; i < numServos; i++) {
    setTarget(i, homePos[i]);
  }
  
}

void resetCommandTimeout() {
  tic.resetCommandTimeout();
}
void delayWhileResettingCommandTimeout(uint32_t ms) {
  uint32_t start = millis();
  do{
    resetCommandTimeout();
  } while ((uint32_t)(millis() - start) <= ms);
}
void waitForPosition(int32_t targetPosition){
  	do{
		resetCommandTimeout();
  	} while (tic.getCurrentPosition() != targetPosition);
}

// Smoothly move each servo toward its target
void updateServos() {
  for (uint8_t i = 0; i < numServos; i++) {  
    if(abs(smoothPos[i] - targetPos[i]) < 1) {
      smoothPos[i] = targetPos[i];
      continue;
    }
    smoothPos[i] = (1 - alpha[i]) * smoothPos[i] + alpha[i] * targetPos[i];
    pwm.writeMicroseconds(i, (int)smoothPos[i]);
  }
}

void loop() {
  //Get a reading from the controller and place it into variables
  Ncon.update();
  int L = Ncon.L();
  int R = Ncon.R();
  // int x = Ncon.axis_x();
  // int y = Ncon.axis_y();
  int D_left = Ncon.D_left();
  int D_right = Ncon.D_right();
  int D_up = Ncon.D_up();
  int D_down = Ncon.D_down();
  int C_left = Ncon.C_left();
  int C_right = Ncon.C_right();
  int C_up = Ncon.C_up();
  int C_down = Ncon.C_down();
  int z = Ncon.Z();
  int a = Ncon.A();
  // int b = Ncon.B();

  //Button Commands
  if (L){
    posStepper = posStepper + stepInc;
    tic.setTargetPosition(posStepper);
    waitForPosition(posStepper);  
    delayWhileResettingCommandTimeout(50);
  }
  else if (R){
    posStepper = posStepper - stepInc;
    tic.setTargetPosition(posStepper);
    delayWhileResettingCommandTimeout(50);
  }
  //Shoulder movement
  if (D_up){
    setTarget(shoulder1, targetPos[shoulder1] + servoInc[shoulder1]);
    setTarget(shoulder2, targetPos[shoulder2] - servoInc[shoulder2]);
  }
  else if (D_down){
    setTarget(shoulder1, targetPos[shoulder1] - servoInc[shoulder1]);
    setTarget(shoulder2, targetPos[shoulder2] + servoInc[shoulder2]);
  }
  //Elbow movement
  if (D_left){
    setTarget(elbow, targetPos[elbow] - servoInc[elbow]);
  }
  else if (D_right){
    setTarget(elbow, targetPos[elbow] + servoInc[elbow]);
  }
  //Wrist1 movement
  if (C_right){
    setTarget(wrist1, targetPos[wrist1] + servoInc[wrist1]);
  }
  else if (C_left){
    setTarget(wrist1, targetPos[wrist1] - servoInc[wrist1]);
    
  }
  //Wrist2 movement
  if (C_up){
    setTarget(wrist2, targetPos[wrist2] + servoInc[wrist2]);
  }
  else if (C_down){
    setTarget(wrist2, targetPos[wrist2] - servoInc[wrist2]);
  }
  //Claw movement
  if(z == 0 && smoothPos[claw] > 0){
    setTarget(claw, targetPos[claw] - servoInc[claw]);
  }
  else if(z){
    setTarget(claw, targetPos[claw] + servoInc[claw]);
  }
  //Move servos to new positions
  updateServos();
  delay(10);

  if(a){
    delay(2000);
    if(a){
      for(uint8_t i = 0; i < numServos; i++) {
        setTarget(i, homePos[i]);
      }
    }
  }

  //Serial print data
  // Serial.print("Base: ");
  // Serial.print(posStepper);
  // Serial.print("\t");
  // Serial.print("Shoulder 1 & 2:");
  // Serial.print(smoothPos[0]);
  // Serial.print("\t");
  // Serial.print(smoothPos[1]);
  // Serial.print("\t");
  // Serial.print("Elbow: ");
  // Serial.print("\t");
  // Serial.print(smoothPos[2]);
  // Serial.print("\t");
  // Serial.print("Wrist1: ");
  // Serial.print(smoothPos[3]);
  // Serial.print("\t");
  // Serial.print("Wrist2: ");
  // Serial.print(smoothPos[wrist2]);
  // Serial.print("\t");
  // Serial.print("Claw:");
  // Serial.println(smoothPos[claw]);

  // Ncon.print_N64_status();
}
