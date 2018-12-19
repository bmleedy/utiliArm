#include "ServoAxis.h"
#include "esp_log.h"

#define SERVOAXIS_TAG "ServoAxis.cpp"

ServoAxis::ServoAxis(uint8_t max_degrees,
	  uint8_t min_degrees,
	  uint8_t start_degrees,
	  gpio_num_t dio_pin) : RobotAxis(max_degrees,
					  min_degrees,
					  start_degrees) {

  dio_pin_ = dio_pin;
  servo_ = new servoControl();
  servo_->attach(dio_pin_);
  go_to(start_degrees);
}


bool ServoAxis::go_to(uint8_t position_deg) {
  this->command_deg_ = position_deg;

  // Check input
  if(command_deg_ < min_deg_)
    command_deg_ = min_deg_;
  else if(command_deg_ > max_deg_)
    command_deg_ = max_deg_;
  
  // Warn if I coerced the value
  if(command_deg_ != position_deg)
    ESP_LOGW(SERVOAXIS_TAG,
	     "Command out of range to axis on pin %d: value: %d \n Setting to %d",
	     dio_pin_, position_deg, command_deg_);

  // write command
  servo_->write(command_deg_);
  position_ = 1000 * command_deg_; //servos are open loop
  
  return true;
}

bool ServoAxis::go_to(uint8_t position_deg, uint16_t speed) {
  return go_to(position_deg); //todo: support servo speed control
}

bool ServoAxis::halt() {
  // servos are open loop, so can't stop them
  return true;  
}
