#include <Arduino.h>

#define FORWARD 1
#define REVERSE 0
#define INVALID 2

// ---------------------------------------------------------------------------
// Motor classe
// ---------------------------------------------------------------------------

class Motor
{
 private:
   byte PWM_pin;
   byte FWD_pin;
   byte REV_pin;
   boolean _rotating;
   byte _direction;
   byte _speed;
 public:
   Motor(byte pin1, byte pin2, byte pin3);
   void rotate(byte direction, byte speed);
   void stop(boolean graduate = false);
   void setSpeed(byte speed);
   void setDirection(byte direction);
   byte getSpeed();
   byte getDirection();
   boolean isRotating();
};
