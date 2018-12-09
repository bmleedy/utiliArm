/* Blink Example

  Based on https://github.com/espressif/esp-idf/blob/master/examples/get-started/blink/main/blink.c

*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "servoControl.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define STEPPER_STEP_PIN GPIO_NUM_27
#define STEPPER_DIRECTION_PIN GPIO_NUM_26
#define STEP_PERIOD_MS 25
#define STEPS_TO_SWEEP 400

#define SERVO_OUTPUT_PIN GPIO_NUM_17
#define SERVO_SWEEP_DEGREES 100
#define SERVO_OFFSET 90

void motion_test_task(void *pvParameter)
{
  // Configure the GPIO pin for the servo
  servoControl myServo;
  myServo.attach(SERVO_OUTPUT_PIN);
  //Defaults: myServo.attach(pin, 400, 2600, LEDC_CHANNEL_0, LEDC_TIMER0);
  // to use more servo set a valid ledc channel and timer
  myServo.write(0);  // sero out the servo


  
  /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
     muxed to GPIO on reset already, but some default to other
     functions and need to be switched to GPIO. Consult the
     Technical Reference for a list of pads and their default
     functions.)
  */

  //initialize state for the step pin and the direction
  bool direction_state = false;
  int servo_command = SERVO_SWEEP_DEGREES / 2; //degrees

  // Select pad as a gpio function from IOMUX
  gpio_pad_select_gpio(STEPPER_STEP_PIN);
  gpio_pad_select_gpio(STEPPER_DIRECTION_PIN);

  /* Set the GPIO as a push/pull output */
  gpio_set_direction(STEPPER_STEP_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(STEPPER_DIRECTION_PIN, GPIO_MODE_OUTPUT);


  while(1) {

    //todo: crash or failu build if STEP_PERIOD_MS is <= 1.

    // Set direction pin
    gpio_set_level(STEPPER_DIRECTION_PIN, direction_state);

    // Command the servo
    myServo.write(servo_command+SERVO_OFFSET);
    
    // Step STEPS_TO_SWEEP times
    for( int i=0; i < STEPS_TO_SWEEP; i++){
      //go high for 1 ms
      gpio_set_level(STEPPER_STEP_PIN, 1);
      vTaskDelay(1 / portTICK_PERIOD_MS);

      //go low for the remainder of the period
      gpio_set_level(STEPPER_STEP_PIN, 0);
      vTaskDelay((STEP_PERIOD_MS - 1) / portTICK_PERIOD_MS);
    }

    // Reverse direction state
    direction_state = !direction_state;
    servo_command = -servo_command;
    printf("Reversing Servo Direction........%d\n",servo_command);
  }
}

extern "C" void app_main()
{
    xTaskCreate(&motion_test_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
