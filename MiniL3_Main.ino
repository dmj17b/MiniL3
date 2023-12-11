
#include <Encoder.h>
#include <PulsePosition.h>
#include "TeensyTimerTool.h"
using namespace TeensyTimerTool;

// Input pins:
#define Hip_INA 14
#define Hip_INB 15
#define Hip_ENCA 4
#define Hip_ENCB 5

#define Knee_INA 22
#define Knee_INB 23
#define Knee_ENCA 0
#define Knee_ENCB 1

#define Wheel_INA 18
#define Wheel_INB 19

//DEBUG:
bool debug = false;

// General control variables
int CPR = 12; // Counts per revolution of encoders
int maxInput = 500;
int maxPWM = 255;

// Timer that will call control functions
PeriodicTimer controlTimer;
// RC receiver input
PulsePositionInput RCX(FALLING);

// Hip Control Variables
float hipK = 15;
float hipD = 10;
float lastHipPos = 0;
float hipGearRatio = 379.17*12*40/16;
volatile int hipInput;

// Knee Control Variables
float kneeK = 50;
float kneeD = 5;
float currKneePos = 0;
float lastKneePos = 0;
float kneeGearRatio = 379.17*12*48/20;
volatile int kneeInput = 0;
volatile int kneePWM;

// Defining encoder objects
Encoder hipEnc(Hip_ENCA, Hip_ENCB);
Encoder kneeEnc(Knee_ENCA,Knee_ENCB);


// Desired control inputs
float desKneePos = 0;
float desHipPos = 360;
float desWheelVel = 0;






/************************************ SETUP ****************************************/
void setup() {
  Serial.begin(9600); // Begin serial communications
  analogWriteResolution(8); // Set analog write resolution (teensy is capable of higher)
  RCX.begin(12);  // Start PPM reader on pin 12
  controlTimer.begin(controlFunc,5000); // Set up control timer to call "controlFunc" every 5000 microseconds
}
















/**************************** Main Control Loop ********************************/
void loop() {
  // put your main code here, to run repeatedly:
  
  // Read safety trigger
  int safety = RCX.read(5);
  if(debug){
    Serial.print("Hip motor position: ");
    Serial.println(360*hipEnc.read()/hipGearRatio);
    Serial.print("Desired hip position: ");
    Serial.println(desHipPos);
    Serial.print("Hip Motor Command: ");
    Serial.println(hipInput);
  }

// Safety off:
  if(safety>1500){

    // With safety off, begin wheel control
    controlTimer.start();

    // Map joystick input to 
    desHipPos = mapfloat(RCX.read(3),999,1999,-90,90);

    // Map joystick input to a knee velocity
    float desKneeVel = mapfloat(RCX.read(1),999,1988,-5,5);
    
    // Apply a controller deadzone for knee velocity
    if(desKneeVel<=0.1 && desKneeVel >= -0.1)desKneeVel=0;
    // Increment desired knee position based on input velocity
    desKneePos+=0.0002*desKneeVel;

    // Map joystick input to wheel motor input
    desWheelVel = mapfloat(RCX.read(2),999,1988,-500,500);

    

  }
// Safety on:
  else{
    // Safety is on, so stop calling the motor control function
    controlTimer.stop();

    // Force motors into coasting mode
    Serial.println("Safety on, motors off!");
    analogWrite(Hip_INA,0);
    analogWrite(Hip_INB,0);
    analogWrite(Knee_INA,0);
    analogWrite(Knee_INB,0);
    analogWrite(Wheel_INA,0);
    analogWrite(Wheel_INB,0);
  }

}






/******************************* FUNCTIONS *********************************/

// Function to be called for control timer
void controlFunc(){
  hipControl();
  kneeControl();
  wheelControl();
}

// PD control for knee
void kneeControl(){
  // Calculate value of knee joint position in degrees
  float kneePos = 360*kneeEnc.read()/kneeGearRatio;

  // Apply PD control law to get some arbitrary knee input
  kneeInput = kneeK*(desKneePos-kneePos) + kneeD*(-(kneePos-lastKneePos));

  // Constrain knee inputs to be between -maxInput and maxInput
  if(kneeInput>maxInput) kneeInput = maxInput;
  if(kneeInput<-maxInput) kneeInput = -maxInput;

  // Map input to PWM signals that we can actually send to motor
  kneePWM = map(kneeInput,-maxInput,maxInput,-maxPWM,maxPWM);

  // If knee input is negative, turn motor CW
  if(kneeInput<0){
    analogWrite(Knee_INA,0);
    analogWrite(Knee_INB,abs(kneePWM));
  }

  // If knee input is positive, turn motor CCW
  else if(kneeInput>0){
    analogWrite(Knee_INB,0);
    analogWrite(Knee_INA,abs(kneePWM));
  }

  // Reset last knee position
  lastKneePos = kneePos;
}

// PD control for hip
void hipControl(){
  // Calculate value of hip joint position in degrees
  float hipPos = 360*hipEnc.read()/hipGearRatio;

  // Apply PD control law to get some arbitrary hip input
  hipInput = hipK*(desHipPos-hipPos) + hipD*(-(hipPos-lastHipPos));

  // Constrain hip inputs to be between -maxInput and maxInput
  if(hipInput>maxInput) hipInput = maxInput;
  if(hipInput<-maxInput) hipInput = -maxInput;

  // Map input to PWM signals that we can actually send to the motor
  int hipPWM = map(hipInput,-maxInput,maxInput,-maxPWM,maxPWM);

  // If input is negative, turn motor CW
  if(hipInput<0){
    analogWrite(Hip_INA,0);
    analogWrite(Hip_INB,abs(hipPWM));
  }
  
  // If input is positive, turn motor CCW
  else if(hipInput>0){
    analogWrite(Hip_INB,0);
    analogWrite(Hip_INA,abs(hipPWM));
  }

  // Reset last knee position
  lastHipPos = hipPos;

}

// Function to send wheel control commands
void wheelControl(){
  int brakeTol = 5; // Input braking tolerance

  // Mapping desired wheel velocity to a wheel PWM signal
  int wheelPWM = map(desWheelVel,-maxInput,maxInput,-maxPWM,maxPWM);

  // If wheel control is negative and out of braking range, send motor inputs
  if(wheelPWM>brakeTol){
    analogWrite(Wheel_INA,0);
    analogWrite(Wheel_INB,abs(wheelPWM));
  }
  // If wheel control is negative and out of braking range, send motor inputs
  else if(wheelPWM<-brakeTol){
    analogWrite(Wheel_INB,0);
    analogWrite(Wheel_INA,abs(wheelPWM));
  }
  // If close to zero input, brake motors
  else{
    analogWrite(Wheel_INA,150);
    analogWrite(Wheel_INB,150);
  }

}

// Helper function to interpolate between float values:
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
