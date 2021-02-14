//https://github.com/adidoo/arduino-sbus Forked from https://github.com/george-hawkins/arduino-sbus
#include <SBUS.h>
//https://www.airspayce.com/mikem/arduino/AccelStepper/
//https://www.arduino.cc/reference/en/libraries/accelstepper/
#include <AccelStepper.h>
#include <limits.h>
#include <math.h>

// Index of Joint in next arrays
#define RB (0)
#define A1 (1)
#define A2 (2)
#define RH (3)
#define A3 (4)
#define MAX_JOINT (5)

// Joint 1 - Rotation Base (On E0)
// Left = negative, Right = positive
// 4000 = 1/4 de tour
#define RB_STEP_PIN         26
#define RB_DIR_PIN          28
#define RB_ENABLE_PIN       24
//#define RB_MIN_STEP         -100
//#define RB_MAX_STEP         100

// Joint 2 - Arm n°1 (On X)
// Up = negative, Down = positive
#define A1_STEP_PIN         54
#define A1_DIR_PIN          55
#define A1_ENABLE_PIN       38
#define A1_MIN_STEP        -2000
#define A1_MAX_STEP         2000

// Joint 3 - Arm n°2 (On Y)
// One arm turn = 92800 steps
// Up = negative, Down = positive
#define A2_STEP_PIN         60
#define A2_DIR_PIN          61
#define A2_ENABLE_PIN       56
#define A2_MIN_STEP        -29000
#define A2_MAX_STEP         29000

// Joint 4 - Rotation hand (On E1)
// One arm/axe turn = 1600 steps
#define RH_STEP_PIN        36
#define RH_DIR_PIN         34
#define RH_ENABLE_PIN      30
#define RH_MIN_STEP       -1600
#define RH_MAX_STEP        1600

// Joint 5 - Arm n°3 (On Z)
// Motor: Nema14 stepper motor 14HS3515
// Angle: 1.8 deg/step
// Current: 1.2A
// One axe turn = 1600 steps
// One arm turn = 7200 steps (Ratio = 4,5)
// Up = positive, Down = negative
#define A3_STEP_PIN         46
#define A3_DIR_PIN          48
#define A3_ENABLE_PIN       62
#define A3_MIN_STEP        -2000
#define A3_MAX_STEP         2000


//SBUS serial link is over Serial2 (TX=16, RX=17, TTL 5V)
// Used with a FrSky R-XSR soldered on inverted SBUS pad
SBUS sbus(Serial2);

// SBUS Chanel are 1 to 6. 0 is unused
// 1 = Left UP/DOWN
// 2 = Right LEFT/RIGHT
// 3 = Right UP/DOWN
// 4 = Left LEFT/RIGHT
// 5 = Left Rotary
// 6 = Right Rotary
int ChannelJointMap[] = {4, 1, 3, 2, 5};
int ChannelMin[]      = {0, 172, 172, 172, 172, 172, 172};
int ChannelCenter[]   = {0, 995, 995, 997, 992, 992, 992};
int ChannelMax[]      = {0, 1811, 1811, 1811, 1811, 1811, 1811};
int ChannelDeadZone[] = {0, 20, 10, 10, 10, 10, 10};

int JointMaxSpeed[]      = {3000, 3000, 10000, 1000, 1000};
int JointAcceleration[]  = {1000, 1000,  4000, 1000, 1000};
int JointSpeed[]  =        {0, 0, 0, 0, 0};

AccelStepper joint[] = {
  AccelStepper(AccelStepper::DRIVER, RB_STEP_PIN, RB_DIR_PIN), // Rotation base
  AccelStepper(AccelStepper::DRIVER, A1_STEP_PIN, A1_DIR_PIN), // Arm 1
  AccelStepper(AccelStepper::DRIVER, A2_STEP_PIN, A2_DIR_PIN), // Arm 2
  AccelStepper(AccelStepper::DRIVER, RH_STEP_PIN, RH_DIR_PIN), // Rotation Arm 3 
  AccelStepper(AccelStepper::DRIVER, A3_STEP_PIN, A3_DIR_PIN)  // Arm 3
};

void setup() {
  
  Serial.begin(115200); // For Computer monitoring and debug
  sbus.begin();
  
  // Configure each stepper
  for(int i = 0; i< MAX_JOINT; i++)
  {
    joint[i].setMaxSpeed(JointMaxSpeed[i]);
    joint[i].setAcceleration(JointAcceleration[i]); //(Not used in speed mode)
  }
}

// Scale the S.BUS channel values into the range [-100%, 100%] for use as motor command.
int getChannel(int channel) {
  int value = sbus.getChannel(channel);
  //Serial.println(value);
  
  // If out of limits, ==> 0%
  if (value < ChannelMin[channel]) {
    value = ChannelCenter[channel];
  }
  if (value > ChannelMax[channel]) {
    value = ChannelCenter[channel];
  }
  
  // Value centered on 0
  float fres = (float) (value - ChannelCenter[channel]);
  // Normaly constant, but in case of any tunning
  float fdelta = (float) min(abs(ChannelCenter[channel] - ChannelMin[channel]), abs(ChannelCenter[channel] - ChannelMax[channel]));
  // Range -100% to 100%
  fres = fres / fdelta * 100.0;

  // Limit in case of min/max variation
  if (fres < -100.0) {
    fres = -100.0;
  }
  if (fres > 100.0) {
    fres = 100.0;
  }
  //Dead zone
  if (abs(fres) < ChannelDeadZone[channel]) {
    fres = 0.0;
  }
  
  return fres;
}

// This is timer2 which triggers ever 1ms and processes the incoming SBUS datastream.
ISR(TIMER2_COMPA_vect)
{
  sbus.process();
}

// Main loop
void loop() {

  for(int i = 0; i< MAX_JOINT; i++)
  {
    JointSpeed[i] = getChannel(ChannelJointMap[i]);
    joint[i].setSpeed(JointSpeed[i]*10);
    Serial.print(joint[i].currentPosition());
    Serial.print(", ");
  }
  Serial.println();
  for(int i = 0; i< MAX_JOINT; i++)
  {
    joint[i].runSpeed();
  }
}
