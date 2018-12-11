/* Blink Example

  Based on https://github.com/espressif/esp-idf/blob/master/examples/get-started/blink/main/blink.c

*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <tcpip_adapter.h>
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "lwip/inet.h"
#include "lwip/ip4_addr.h"

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

ip_addr_t ip_Addr;
ip4_addr_t ip;
ip4_addr_t gw;
ip4_addr_t msk;
bool bConnected = false;
bool bDNSFound = false;

esp_err_t event_handler(void *ctx, system_event_t *event)
{
	printf("event_handler: received event [%X]\n", event->event_id);

    switch( event->event_id ){
    	case SYSTEM_EVENT_STA_GOT_IP :
    		/*
			ip = event->event_info.got_ip.ip_info.ip;
			gw = event->event_info.got_ip.ip_info.gw;
			msk = event->event_info.got_ip.ip_info.netmask;
			bConnected = true;

			printf("Got IP: %s\n", inet_ntoa( ip ) );
			printf("Net mask: %s\n", inet_ntoa( msk ) );
			printf("Gateway: %s\n", inet_ntoa( gw ) );
			*/
			break;
    	case SYSTEM_EVENT_STA_START :
    		printf("Station start event.\n");
    		esp_wifi_connect();
    		break;
    	case SYSTEM_EVENT_STA_CONNECTED :
    		printf("Station Connected!\n");
    		break;
    	case SYSTEM_EVENT_STA_DISCONNECTED :
    		printf("Station Disconnected!\n");
    		esp_wifi_connect();
    		//xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    		break;
    	default:
    		printf("event_handler: event %d", event->event_id);
    }

    return ESP_OK;
}


void initialize_wifi()
{
	printf("wifi flash init...\n");
	nvs_flash_init();
	printf("wifi adapter init...\n");
	tcpip_adapter_init();
	printf("wifi event loop init\n");
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	printf("wifi init\n");
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	printf("wifi set storage\n");
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	printf("wifi set mode\n");
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	//create station config
	wifi_config_t sta_config;

	strncpy((char *)sta_config.sta.ssid,CONFIG_ESP_WIFI_SSID,32);
	strncpy((char *)sta_config.sta.password,CONFIG_ESP_WIFI_PASSWORD,32);
	sta_config.sta.bssid_set = false;
	printf("wifi set config\n");
	ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
	printf("wifi start\n");
	ESP_ERROR_CHECK( esp_wifi_start() );
	printf("wifi connect\n");
	ESP_ERROR_CHECK( esp_wifi_connect() );

}





void motion_test_task(void *pvParameter)
{

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
	printf("level set\n");
    gpio_set_level(STEPPER_DIRECTION_PIN, direction_state);

    // Command the servo
    printf("servo write\n");
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
	printf("initializing WiFi..\n");
	initialize_wifi();

    //xTaskCreate(&motion_test_task, "motion_test", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
