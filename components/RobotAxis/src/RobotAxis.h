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
  virtual bool go_to(uint8_t position_deg) {return false;}
  virtual bool go_to(uint8_t position_deg, uint16_t speed) {return false;}
  virtual bool go_to_blocking(uint8_t position_deg){return go_to(position_deg);}
  virtual bool halt() {return false;}
  int32_t get_position() {return position_;}


protected:
  uint8_t max_deg_;
  uint8_t min_deg_;
  int32_t position_ = 0; //millidegrees
  uint8_t command_deg_;
};

#endif
