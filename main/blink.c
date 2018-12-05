/* Blink Example

  Based on https://github.com/espressif/esp-idf/blob/master/examples/get-started/blink/main/blink.c

*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define STEPPER_STEP_PIN 5
#define STEPPER_DIRECTION_PIN 6
#define STEP_PERIOD_MS 10
#define STEPS_TO_SWEEP 50

void blink_task(void *pvParameter)
{
  /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
     muxed to GPIO on reset already, but some default to other
     functions and need to be switched to GPIO. Consult the
     Technical Reference for a list of pads and their default
     functions.)
  */

  //initialize state for the step pin and the direction
  bool direction_state = false;


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

    // Step STEPS_TO_SWEEP times
    for( int i=0; i < STEPS_TO_SWEEP; i++){
      //go high for 1 ms
      gpio_set_level(BLINK_GPIO, 1);
      vTaskDelay(1 / portTICK_PERIOD_MS);

      //go low for the remainder of the period
      gpio_set_level(BLINK_GPIO, 0);
      vTaskDelay((STEP_PERIOD_MS - 1) / portTICK_PERIOD_MS);
    }

    // Reverse direction state
    direction_state != direction_state;

  }
}

void app_main()
{
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
