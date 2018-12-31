#include "StepperAxis.h"
#include "esp_log.h"  // NOLINT(*) - for spurious linter error

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
  int32_t motion_needed_centideg = params->destination - *params->position;
  int32_t motion_needed_steps =
      motion_needed_centideg * params->steps_per_rev / 36000;

  ESP_LOGI(STEPPERAXIS_TAG,
      "move(): Move command received. Desired: %d centideg "
          "Actual: %d centideg Delta: %d steps",
          params->destination, *params->position,
      motion_needed_steps);

  // TRUE direction bit for positive motion
  bool direction_bit = (motion_needed_steps > 0);
  ESP_LOGI(STEPPERAXIS_TAG,
        "move(): Direction set: %d on pin %d. ", direction_bit,
        params->direction_pin);

  unsigned int direction_value;
  if (params->direction_flag)
    direction_value = direction_bit ? 0 : 1;
  else
    direction_value = direction_bit ? 1 : 0;

    ESP_ERROR_CHECK(gpio_set_level(params->direction_pin, direction_value));

  // todo: implement speed
  if (params->speed == 0 || params->steps_per_rev == 0)
    ESP_LOGE(STEPPERAXIS_TAG, "move(): invalid speed or steps per rev: %d, %d",
        params->speed, params->steps_per_rev);

  motion_needed_steps = abs(motion_needed_steps);

  ESP_LOGI(STEPPERAXIS_TAG,
      "move(): moving %d steps in direction %d with step period %dms",
      motion_needed_steps, direction_value, STEPPERAXIS_MOVE_PERIOD_TICKS+1);


  for (int32_t i = 0; i < motion_needed_steps; i++) {
    // DIO High
    ESP_ERROR_CHECK(gpio_set_level(params->step_pin, 1));
    // Wait one tick (1ms)
    vTaskDelay(1);
    // DIO Low
    ESP_ERROR_CHECK(gpio_set_level(params->step_pin, 0));

    // Indicate that we've moved a step
    if (motion_needed_steps >= 0)
      *params->position = *params->position + 36000 / params->steps_per_rev;
    else
      *params->position = *params->position - 36000 / params->steps_per_rev;

    // todo: change this to an ISR that disables the motor on the rising edge
    if (i > 5 && gpio_get_level(params->limit_pin) == 1) {
      ESP_LOGE(STEPPERAXIS_TAG, "move(): Limit switch hit"
          " during movement! Stopping.");
      if (params->am_a_task)
        vTaskDelete(NULL);
      else
        break;
    }

    // Wait half of period
    vTaskDelay(STEPPERAXIS_MOVE_PERIOD_TICKS);  // todo: implement speed


  }

  // todo: this is a bit of a fudge, but reduces potential rounding error
  *params->position = params->destination;
  // destroy myself if I'm being run as a task

  ESP_LOGI(STEPPERAXIS_TAG, "move(): movement complete.");
  if (params->am_a_task)
    vTaskDelete(NULL);

  // todo: check for the limit pin and set the zero position
  //  if it trips on this step
}

StepperAxis::StepperAxis(uint8_t max_degrees, uint8_t min_degrees,
    uint8_t start_degrees, uint32_t motor_steps_per_rev,
    gpio_num_t step_dio_pin, gpio_num_t direction_dio_pin,
    gpio_num_t limit_dio_pin, unsigned int home_direction) :
    RobotAxis(max_degrees, min_degrees, start_degrees) {
  // GPIO
  motor_steps_per_rev_ = motor_steps_per_rev;
  step_dio_pin_        = step_dio_pin;
  direction_dio_pin_   = direction_dio_pin;
  limit_dio_pin_       = limit_dio_pin;
  home_direction_      = home_direction;

  gpio_pad_select_gpio(step_dio_pin_);
  gpio_pad_select_gpio(direction_dio_pin_);
  gpio_pad_select_gpio(limit_dio_pin_);
  gpio_set_direction(step_dio_pin_, GPIO_MODE_OUTPUT);
  gpio_set_direction(direction_dio_pin_, GPIO_MODE_OUTPUT);
  gpio_set_direction(limit_dio_pin_, GPIO_MODE_INPUT);
  gpio_pullup_en(limit_dio_pin_);

  // Home
  if (find_home(home_direction_))
    go_to(start_degrees);
  else
    ESP_LOGE(STEPPERAXIS_TAG, "StepperAxis(%d,%d,%d,%d,%d,%d,%d,%d):"
        " Limit not found! Axis disabled.",
        max_degrees, min_degrees, start_degrees, motor_steps_per_rev,
        step_dio_pin, direction_dio_pin, limit_dio_pin, home_direction);
}

bool StepperAxis::find_home(unsigned int direction) {
  ready_ = false;  // clear any previous "ready"

  // don't try to find home farther than 180 degrees of steps
  for (int32_t i = 0; i < motor_steps_per_rev_ / 2; i++) {
    // check limit switch before trying to move
    if (gpio_get_level(limit_dio_pin_)) {
      ESP_LOGI(STEPPERAXIS_TAG, "find_home(%d): Limit found!"
          " Setting position to zero.",
          direction);
      position_ = 0;
      ready_ = true;  // this is the only place where
                      // we automatically set ready to true
      break;
    }
    // take one more step in the *negative* direction
    step(direction);

    vTaskDelay(STEPPERAXIS_HOMING_PERIOD_TICKS);

    if ( (i % 100) == 0 ) {
      ESP_LOGI(STEPPERAXIS_TAG, "find_home(%d): Seeking limit."
          "Step[%d], DIO[%d]",
          direction, i, gpio_get_level(limit_dio_pin_));
    }
  }

  return ready_;  // true=success; false=failure
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
  ESP_LOGI(STEPPERAXIS_TAG, "go_to(%d,%d): Command received", position_deg,
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
  task_params_.destination = command_deg_ * 100;
  task_params_.position = &position_;
  task_params_.speed = speed;
  task_params_.direction_flag = home_direction_;
  task_params_.direction_pin = direction_dio_pin_;
  task_params_.step_pin = step_dio_pin_;
  task_params_.limit_pin = limit_dio_pin_;
  task_params_.steps_per_rev = motor_steps_per_rev_;
  task_params_.am_a_task = !blocking_;

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


void StepperAxis::step(unsigned int direction) {
  unsigned int servodir = 0;
  if (direction)
    servodir = 1;
  // set direction
  gpio_set_level(direction_dio_pin_, servodir);
  // ESP_LOGD(STEPPERAXIS_TAG, "step(%d): stepping in direction %d",
  //          direction, servodir);
  // perform 1 OS tick step
  gpio_set_level(step_dio_pin_, 1);
  vTaskDelay(1);
  gpio_set_level(step_dio_pin_, 0);
  // (end state should always be zero)
}
