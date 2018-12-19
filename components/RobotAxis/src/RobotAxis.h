#ifndef ROBOTAXIS_H
#define ROBOTAXIS_H

#include "freertos/FreeRTOS.h"

//todo: cpplint
//todo: doxygen

class RobotAxis {

public:
  //constructor
  RobotAxis(uint8_t max_degrees,
	    uint8_t min_degrees,
	    uint8_t start_degrees) : max_deg_(max_degrees),
                                     min_deg_(min_degrees),
                                     command_deg_(start_degrees) {}
  bool go_to(uint8_t position_deg) {return false;}
  bool go_to(uint8_t position_deg, uint16_t speed) {return false;}
  bool go_to_blocking(uint8_t position_deg){return go_to(position_deg);}
  bool halt() {return false;}
  uint8_t get_position() {return position_;}


protected:
  uint8_t max_deg_;
  uint8_t min_deg_;
  int32_t position_ = 0; //millidegrees
  uint8_t command_deg_;
};

#endif
