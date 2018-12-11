/* Blink Example

  Based on https://github.com/espressif/esp-idf/blob/master/examples/get-started/blink/main/blink.c

*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "string.h"

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

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void initialize_wifi()
{
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  //create station config
  wifi_config_t sta_config;

  strncpy((char *)sta_config.sta.ssid,CONFIG_ESP_WIFI_SSID,32);
  strncpy((char *)sta_config.sta.password,CONFIG_ESP_WIFI_PASSWORD,32);
  sta_config.sta.bssid_set = false;
  ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
  ESP_ERROR_CHECK( esp_wifi_start() );
  ESP_ERROR_CHECK( esp_wifi_connect() );
}


void print_ip_address(){    
  uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
  int8_t cBuffer[ 16 ];
  
  FreeRTOS_GetAddressConfiguration( &ulIPAddress,
				    &ulNetMask,
				    &ulGatewayAddress,
				    &ulDNSServerAddress );
  
  /* Convert the IP address to a string then print it out. */
  FreeRTOS_inet_ntoa( ulIPAddress, cBuffer );
  printf( "IP Address: %s\r\n", cBuffer );
  
  /* Convert the net mask to a string then print it out. */
  FreeRTOS_inet_ntoa( ulNetMask, cBuffer );
  printf( "Subnet Mask: %s\r\n", cBuffer );
  
  /* Convert the IP address of the gateway to a string then print it out. */
  FreeRTOS_inet_ntoa( ulGatewayAddress, cBuffer );
  printf( "Gateway IP Address: %s\r\n", cBuffer );
  
  /* Convert the IP address of the DNS server to a string then print it out. */
  FreeRTOS_inet_ntoa( ulDNSServerAddress, cBuffer );
  printf( "DNS server IP Address: %s\r\n", cBuffer );
}




void motion_test_task(void *pvParameter)
{

  // initialize network
  initialize_wifi();
  network_config my_netconfig;

  // Configure the GPIO pin for the servo
  servoControl myServo;
  myServo.attach(SERVO_OUTPUT_PIN);
  //Defaults: myServo.attach(pin, 400, 2600, LEDC_CHANNEL_0, LEDC_TIMER0);
  // to use more servo set a valid ledc channel and timer
  myServo.write(0);  // zero out the servo

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

    //todo: crash or fail to build if STEP_PERIOD_MS is <= 1.

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

    // Print my IP Address
    print_ip_address();

  }
}



extern "C" void app_main()
{
    xTaskCreate(&motion_test_task, "motion_test", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
