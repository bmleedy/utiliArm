#ifndef INC_STEPPERAXIS
#define INC_STEPPERAXIS

#include "ArmAxis.h"
#include "Arduino.h"

struct StepperStatus{
  int8_t direction = 0;     // -1 negative, 0 stop, 1 positive
  int16_t raw_counts = 0;   // raw A to D counts
};

class StepperAxis : public ArmAxis
{
  public:
  StepperAxis(int16_t default_position, // default position
            uint8_t potentiometer_pin,  // analog input for potentiometer position
            uint8_t step_pin    ,       // digital output for stepper steps
            uint8_t dir_pin     ,       // digital output for step direction
            int16_t max_position,     // max position (tenths)
            int16_t zero_counts,       // what counts equals zero position for the axis
            int16_t min_position);    // min position (tenths)
  bool run_axis();

  private: 
  uint8_t potentiometer_pin;   // analog input pin
  uint8_t step_pin;    // pulse for one step
  uint8_t dir_pin;     // set direction of motion
  int16_t zero_counts;  //

  uint32_t last_step_us;  //micros() output from last step by the controller
  uint8_t last_step_pin_state=LOW;

  StepperStatus status;
  
  bool zero_energy();
  int16_t read_rate();  
  int16_t read_position();
  void move_constant_velocity(int8_t direction);

  void print();
};

#endif
