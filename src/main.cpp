#include <Arduino.h>
#include <AccelStepper.h>
#include <ServoInput.h>
#include "I2Cdev.h"
//#include <MsTimer2.h>
#include <FlexiTimer2.h>
#include "config.h"
#include "mpu.h"
#include <PID_v1.h>

AccelStepper stepperLeft = AccelStepper(motorInterfaceType, stepPinLeft, dirPinLeft);
AccelStepper stepperRight = AccelStepper(motorInterfaceType, stepPinRight, dirPinRight);

ServoInputPin<CH1> servoGas;
ServoInputPin<CH2> servoTurn;


//Define Variables we'll be connecting to
double input, output;
double originalSetpoint =0;
double setpoint = originalSetpoint;
//adjust these values to fit your own design
double Kp = 55;   
double Ki = 0;
double Kd = 0.75;

//Specify the links and initial tuning parameters
PID pid(&input, &output, &setpoint, Kp, Ki, Kd,P_ON_E, DIRECT);


// ================================================================
// ===               TIMER Interrupt                            ===
// ================================================================

ISR(TIMER2_COMPA_vect) {
//  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
//}
//void timerInterrupt() {
  stepperLeft.runSpeed();
  stepperRight.runSpeed();

  //  stepperLeft.run();
  //stepperRight.run();
}

void setupTimer2() {
  noInterrupts();
  // Clear registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // 25000 Hz (16000000/((4+1)*128))
  OCR2A = 4;
  // CTC
  TCCR2A |= (1 << WGM21);
  // Prescaler 128
  TCCR2B |= (1 << CS22) | (1 << CS20);
  // Output Compare Match A Interrupt Enable
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}


void setup() {
  Serial.begin(115200);
  
  //Stepper init
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);

  stepperLeft.setMaxSpeed(300000);
  stepperRight.setMaxSpeed(300000);
  stepperLeft.setAcceleration(20000);
  stepperRight.setAcceleration(20000);

  //Rc input init
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);

  initMPU();

  //Timer Init
  //FlexiTimer2::set(1,0.00025, timerInterrupt); // trigger an interrupt every 25ms 
  //FlexiTimer2::start();    // start interrupt  

//setupTimer2();

 //setup PID
 pid.SetMode(AUTOMATIC);
 //pid.SetSampleTime(10);
 pid.SetOutputLimits(-5000, 5000);
}

//90 > 1000
//1 > 1

int counter = 0;

void loop() {

  /*loopMPU();

  float currentAngle = getLatestYawPitchRoll().pitch;
  input = currentAngle;
  pid.Compute();*/

/*if (currentAngle > 45 || currentAngle < -45)
{
  stepperLeft.setSpeed(0);
    stepperRight.setSpeed(0);
     digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}
else*/
{
  output = 1600;
  stepperRight.setSpeed(-(output));
  stepperLeft.setSpeed(output);
}

  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}