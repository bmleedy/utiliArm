#ifndef COMPONENTS_ROBOTAXIS_SRC_ROBOTAXIS_H_
#define COMPONENTS_ROBOTAXIS_SRC_ROBOTAXIS_H_

#include "freertos/FreeRTOS.h"

//  todo: doxygen

class RobotAxis {
 public:
  RobotAxis(uint8_t max_degrees,
            uint8_t min_degrees,
            uint8_t start_degrees) :
      max_deg_(max_degrees),
      min_deg_(min_degrees),
      command_deg_(start_degrees) {
  }
  virtual ~RobotAxis(){}
  virtual bool go_to(uint8_t position_deg) {
    return false;
  }
  virtual bool go_to(uint8_t position_deg, uint16_t speed) {
    return false;
  }
  virtual bool go_to_blocking(uint8_t position_deg) {
    return go_to(position_deg);
  }
  virtual bool halt() {
    return false;
  }
  int32_t get_position() {
    return position_;
  }

 protected:
  uint8_t max_deg_;       // degrees
  uint8_t min_deg_;       // degrees
  int32_t position_ = 0;  // millidegrees
  uint8_t command_deg_;   // degrees
};

#endif  // COMPONENTS_ROBOTAXIS_SRC_ROBOTAXIS_H_
