#ifndef COMPONENTS_ROBOTAXIS_SRC_SERVOAXIS_H_
#define COMPONENTS_ROBOTAXIS_SRC_SERVOAXIS_H_

#include "RobotAxis.h"
#include "servoControl.h"

// todo: doxygen

class ServoAxis: public RobotAxis {
 public:
    // RobotAxis API
    ServoAxis(uint8_t max_degrees, uint8_t min_degrees, uint8_t start_degrees,
        gpio_num_t dio_pin, int32_t timer_channel = 0);
    ~ServoAxis() {}
    bool go_to(uint8_t position_deg);
    bool go_to(uint8_t position_deg, uint16_t speed);
    bool halt();

    // ServoAxis methods
    bool get_dio_pin() {
      return dio_pin_;
    }

 private:
    gpio_num_t dio_pin_;
    int32_t timer_channel_;
    servoControl * servo_;
};

#endif  // COMPONENTS_ROBOTAXIS_SRC_SERVOAXIS_H_
