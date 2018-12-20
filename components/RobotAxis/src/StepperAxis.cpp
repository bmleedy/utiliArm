#include "StepperAxis.h"
#include "esp_log.h"

#define STEPPERAXIS_DEFAULT_SPEED 100  // centidegrees per second
#define STEPPERAXIS_TAG "StepperAxis.cpp"

// todo: doxygen
// This is the task which sends the pulses to the motor to move it
// todo: this task could be implemented with queues
//    (long running task)
// todo: this task could also be implemented with an ISR
//    (short running task)
extern "C" void move(void * pvParams) {
  stepTaskParameters * params =
      reinterpret_cast<stepTaskParameters *>(pvParams);

  // only performs motion in one direction at a time (no feedback)
  int32_t motion_needed = params->destination - *params->position;

  ESP_LOGI(STEPPERAXIS_TAG,
      "move(): Move command received. Desired: %d millideg "
          "Actual: %d Delta: %d", params->destination, *params->position,
      motion_needed);

  // direction flag will invert direction bit
  bool direction_bit = (motion_needed < 0);
  gpio_set_level(GPIO_NUM_22, direction_bit);

  if (params->speed == 0 || params->steps_per_rev == 0)
    ESP_LOGE(STEPPERAXIS_TAG, "move(): invalid speed or steps per rev: %d, %d",
        params->speed, params->steps_per_rev);

  // how many steps do I need to move?
  int32_t steps_needed = abs(motion_needed * params->steps_per_rev / 360000);

  ESP_LOGD(STEPPERAXIS_TAG,
      "move(): moving %d steps in direction %d with step period %dms",
      steps_needed, direction_bit, 2);

  ESP_LOGD(STEPPERAXIS_TAG, "move(): there are %d ms per tick",
      portTICK_PERIOD_MS);

  for (int32_t i = 0; i < steps_needed; i++) {
    // DIO High
    gpio_set_level(GPIO_NUM_23, 1);

    // Wait half of period
    vTaskDelay(1);  // todo: implement speed

    // DIO Low
    gpio_set_level(GPIO_NUM_23, 0);
    // Indicate that we've moved a step
    if (motion_needed >= 0)
      *params->position = *params->position + 360000 / params->steps_per_rev;
    else
      *params->position = *params->position - 360000 / params->steps_per_rev;

    // Wait half of period
    vTaskDelay(1);  // todo: implement speed
  }

  // todo: this is a bit of a fudge, but reduces potential rounding error
  *params->position = params->destination;
  // destroy myself
  // vTaskDelete(NULL);
}

StepperAxis::StepperAxis(uint8_t max_degrees, uint8_t min_degrees,
    uint8_t start_degrees, uint32_t motor_steps_per_rev,
    gpio_num_t step_dio_pin, gpio_num_t direction_dio_pin,
    gpio_num_t limit_dio_pin, bool home_direction) :
    RobotAxis(max_degrees, min_degrees, start_degrees) {
  // GPIO
  motor_steps_per_rev_ = motor_steps_per_rev;
  step_dio_pin_ = step_dio_pin;
  direction_dio_pin_ = direction_dio_pin;
  limit_dio_pin_ = limit_dio_pin;

  gpio_pad_select_gpio(step_dio_pin_);
  gpio_pad_select_gpio(direction_dio_pin_);
  gpio_pad_select_gpio(limit_dio_pin_);
  gpio_set_direction(step_dio_pin_, GPIO_MODE_OUTPUT);
  gpio_set_direction(direction_dio_pin_, GPIO_MODE_OUTPUT);
  gpio_set_direction(limit_dio_pin_, GPIO_MODE_INPUT);

  // Home
  if (find_home(home_direction_))
    go_to(start_degrees);
}

bool StepperAxis::find_home(bool direction) {
  // todo: move the motor in the homing direction
  // todo: set the ready_ flag for success or failure
  // todo: send message out
  // todo: return the status of the ready flag
  ready_ = false;
  return ready_;
}

bool StepperAxis::go_to_blocking(uint8_t position_deg) {
  blocking_ = true;
  return go_to(position_deg);
}

bool StepperAxis::go_to(uint8_t position_deg) {
  ESP_LOGD(STEPPERAXIS_TAG, "go_to(): Defaulting speed to %d centidegrees/s",
  STEPPERAXIS_DEFAULT_SPEED);
  return go_to(position_deg, STEPPERAXIS_DEFAULT_SPEED);
}

bool StepperAxis::go_to(uint8_t position_deg, uint16_t speed) {
  ESP_LOGD(STEPPERAXIS_TAG, "go_to(%d,%d): Command received", position_deg,
      speed);
  // Always stop previous motion
  halt();

  // if motor is not ready (home found), refuse to move
  if (!ready_) {
    ESP_LOGE(STEPPERAXIS_TAG,
        "Stepper is not ready to move. Ignoring command.");
    return false;
  }

  this->command_deg_ = position_deg;

  // Check input
  if (command_deg_ < min_deg_)
    command_deg_ = min_deg_;
  else if (command_deg_ > max_deg_)
    command_deg_ = max_deg_;

  // Warn if I coerced the value
  if (command_deg_ != position_deg)
    ESP_LOGW(STEPPERAXIS_TAG,
        "Command out of range to axis on pin %d: value: %d", step_dio_pin_,
        position_deg);

  // spin off a task to to x number of steps at y speed in z direction
  task_params_.destination = 1000 * position_deg;
  task_params_.position = &position_;
  task_params_.speed = speed;
  task_params_.direction_flag = home_direction_;
  task_params_.direction_pin = direction_dio_pin_;
  task_params_.step_pin = step_dio_pin_;
  task_params_.steps_per_rev = motor_steps_per_rev_;

  blocking_ = true;  // todo: fixme

  if (blocking_) {
    ESP_LOGD(STEPPERAXIS_TAG, "go_to(): Blocking move.");
    move(reinterpret_cast<void*>(&task_params_));
  } else {
    xTaskCreate(&move, "move_stepper",
        2000,
        reinterpret_cast<void*>(&task_params_),
        5,
        NULL);
  }
  // todo spin off task here

  return true;
}

// Deletes the task performing the current motion.
bool StepperAxis::halt() {
  if (stepper_task_ == NULL) {
    ESP_LOGD(STEPPERAXIS_TAG, "halt(): No motion task to delete");
  } else {
    ESP_LOGD(STEPPERAXIS_TAG, "halt(): Stopping motion task: %d",
        (int) stepper_task_);
    vTaskDelete(stepper_task_);
  }
  stepper_task_ = NULL;
  return true;
}
