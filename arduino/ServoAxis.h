#ifndef INC_SERVOAXIS
#define INC_SERVOAXIS

#include <Servo.h>
#include "ArmAxis.h"

class ServoAxis : public ArmAxis
{
  public:
  ServoAxis(int16_t default_position, // default position
            uint8_t output_pin,       // output pin
            int16_t max_position,     // max position (tenths)
            int16_t zero_angle,       // what servo angle is zero angle for the joint
            int16_t min_position);    // min position (tenths)
  bool run_axis();

  private: 
  uint8_t control_pin;
  int16_t zero_angle;
  Servo servo_control;
  
  bool zero_energy();
  int16_t read_rate();  
  int16_t read_position();

  void print(){}
};

#endif
