#include <Arduino.h>
//#include <ServoInput.h>
//#include "I2Cdev.h"
//#include <MsTimer2.h>
//#include <FlexiTimer2.h>
#include "config.h"
//#include "mpu.h"
//#include <PID_v1.h>
#include "FastAccelStepper.h"

//ServoInputPin<CH1> servoGas;
//ServoInputPin<CH2> servoTurn;


//Define Variables we'll be connecting to
double input, output;
double originalSetpoint =0;
double setpoint = originalSetpoint;
//adjust these values to fit your own design
double Kp = 55;   
double Ki = 0;
double Kd = 0.75;

//Specify the links and initial tuning parameters
//PID pid(&input, &output, &setpoint, Kp, Ki, Kd,P_ON_E, DIRECT);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  
  //Stepper init
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  //Set not used pins as Input
  pinMode(stepPinRightNotUsed, INPUT);
  pinMode(stepPinLeftNotUsed, INPUT);

 engine.init();
  stepper = engine.stepperConnectToPin(stepPinRight);
  if (stepper) {
    stepper->setDirectionPin(dirPinRight);
    stepper->setEnablePin(MOTOR_ENABLE_PIN);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInUs(1500);  // the parameter is us/step !!!
    stepper->setAcceleration(100000);
    stepper->runBackward();
  }
}

void loop() {}