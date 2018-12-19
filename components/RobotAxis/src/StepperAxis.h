#ifndef STEPPERAXIS_H
#define STEPPERAXIS_H


#include "RobotAxis.h"
#include "freertos/task.h"


//todo: cpplint
//todo: doxygen

struct stepTaskParameters{
  int32_t destination; // centidegrees
  int32_t * position;  // centidegrees
  int32_t steps_per_rev;
  int32_t speed;      // centidegrees/s
  bool direction_flag; // true for direction bit high = positive
  gpio_num_t direction_pin;
  gpio_num_t step_pin;
};

class StepperAxis : public RobotAxis {

 public:
  // RobotAxis API
  StepperAxis(uint8_t max_degrees,
	      uint8_t min_degrees,
	      uint8_t start_degrees,
	      uint32_t motor_steps_per_rev,
	      gpio_num_t step_dio_pin,
	      gpio_num_t direction_dio_pin,
	      gpio_num_t limit_dio_pin,
	      bool home_direction);
  bool go_to(uint8_t position_deg);
  bool go_to_blocking(uint8_t position_deg);
  bool go_to(uint8_t position_deg, uint16_t speed);
  bool halt();


  void force_ready(){ready_ = true;}

  // StepperAxis private methods
 private:
  bool find_home(bool direction);
  uint8_t get_position(){return position_;}
  
 private:
  gpio_num_t step_dio_pin_;
  gpio_num_t direction_dio_pin_;
  gpio_num_t limit_dio_pin_;
  bool home_direction_; // true for direction bit high = positive 
  bool ready_ = false; //home found
  TaskHandle_t stepper_task_ = NULL; //NULL if no task
  stepTaskParameters task_params_;
  bool blocking_ = false; //goto calls block
  uint32_t motor_steps_per_rev_ = 200;
};

#endif
