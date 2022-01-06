#define dirPinLeft 7
#define stepPinLeft 10
#define stepPinLeftNotUsed 4

#define dirPinRight 13
#define stepPinRight 9
#define stepPinRightNotUsed 12
#define motorInterfaceType 1
#define MOTOR_ENABLE_PIN 8
#define mpu_interrupt_pin 11

#define CH1 3
#define CH2 2

#ifndef MpuYawPitchRoll_H
#define MpuYawPitchRoll_H
struct MpuYawPitchRoll {
  float yaw, pitch, roll;
};
#endif
