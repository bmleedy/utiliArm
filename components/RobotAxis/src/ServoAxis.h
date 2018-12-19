#ifndef SERVOAXIS_H
#define SERVOAXIS_H


#include "RobotAxis.h"
#include "servoControl.h"

//todo: cpplint
//todo: doxygen

class ServoAxis : public RobotAxis {

public:
  // RobotAxis API
  ServoAxis(uint8_t max_degrees,
	    uint8_t min_degrees,
	    uint8_t start_degrees,
	    gpio_num_t dio_pin);
  bool go_to(uint8_t position_deg);
  bool go_to(uint8_t position_deg, uint16_t speed);
  bool halt();

  // ServoAxis methods
  bool get_dio_pin(){return dio_pin_;}

private:
  gpio_num_t dio_pin_;
  servoControl * servo_;

};

#endif
