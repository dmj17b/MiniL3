
#include <Encoder.h>
#include <PulsePosition.h>

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

// General control variables
int CPR = 12; // Counts per revolution of encoders
int maxInput = 500;
int maxPWM = 255;

// RC receiver input
PulsePositionInput RCX(FALLING);

// Hip Control Variables
float hipK = 15;
float hipD = 10;
float lastHipPos = 0;
float hipGearRatio = 210.59*12*40/16;
volatile int hipInput;

// Knee Control Variables
float kneeK = 50;
float kneeD = 5;
float currKneePos = 0;
float lastKneePos = 0;
float kneeGearRatio = 150.58*12*48/20;
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
  // put your setup code here, to run once:
  Serial.begin(9600);
  analogWriteResolution(8);
  RCX.begin(12);
  

}
















/**************************** Main Control Loop ********************************/
void loop() {
  // put your main code here, to run repeatedly:
  
  // Read safety trigger
  int safety = RCX.read(5);


// Safety off:
  if(safety>1500){
    desHipPos = mapfloat(RCX.read(3),999,1999,-90,90);
    //desKneePos = mapfloat(RCX.read(1),999,1988,-90,90);
    float desKneeVel = mapfloat(RCX.read(1),999,1988,-5,5);
    
    if(desKneeVel<=0.1 && desKneeVel >= -0.1)desKneeVel=0;
    Serial.println(desKneePos);
    desKneePos+=0.0001*desKneeVel;
    desWheelVel = mapfloat(RCX.read(2),999,1988,-500,500);

    

    // Call control function every 5ms
    // (This will eventually be on a timer interrupt)
    if(millis()%5==0){
      hipControl();
      kneeControl();
      wheelControl(desWheelVel);
    }

  }
// Safety on:
  else{
    Serial.println("Safety on, motors off!");
    analogWrite(Hip_INA,0);
    analogWrite(Hip_INB,0);
    analogWrite(Knee_INA,0);
    analogWrite(Knee_INB,0);
    analogWrite(Wheel_INA,0);
    analogWrite(Wheel_INB,0);
  }





}











/********************** FUNCTIONS ********************/

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

void wheelControl(int input){
  int brakeTol = 5;
  int wheelPWM = map(input,-maxInput,maxInput,-maxPWM,maxPWM);

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

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
