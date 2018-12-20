#include "ServoAxis.h"
#include "esp_log.h"

#define SERVOAXIS_TAG "ServoAxis.cpp"

ServoAxis::ServoAxis(uint8_t max_degrees, uint8_t min_degrees,
    uint8_t start_degrees, gpio_num_t dio_pin, int32_t timer_channel) :
    RobotAxis(max_degrees, min_degrees, start_degrees) {
  ESP_LOGD(SERVOAXIS_TAG, "constructor() attaching to gpio pin: %d", dio_pin);
  dio_pin_ = dio_pin;
  timer_channel_ = timer_channel;
  servo_ = new servoControl();
  servo_->attach(dio_pin_, 400, 2600, (ledc_channel_t) timer_channel_);
  go_to(start_degrees);
}

bool ServoAxis::go_to(uint8_t position_deg) {
  ESP_LOGD(SERVOAXIS_TAG, "goto() command received to pin %d: %d", dio_pin_,
      position_deg);

  this->command_deg_ = position_deg;

  // Check input
  if (command_deg_ < min_deg_)
    command_deg_ = min_deg_;
  else if (command_deg_ > max_deg_)
    command_deg_ = max_deg_;

  // Warn if I coerced the value
  if (command_deg_ != position_deg)
    ESP_LOGW(SERVOAXIS_TAG,
        "Command out of range to axis on pin %d: value: %d \n Setting to %d",
        dio_pin_, position_deg, command_deg_);

  // write command
  servo_->write(command_deg_);
  position_ = 1000 * command_deg_;  // servos are open loop

  return true;
}

bool ServoAxis::go_to(uint8_t position_deg, uint16_t speed) {
  return go_to(position_deg);  // todo: support servo speed control
}

bool ServoAxis::halt() {
  // servos are open loop, so can't stop them
  return true;
}
