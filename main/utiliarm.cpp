/*!
 * @file utiliarm.cpp
 * 
 * @mainpage Robotic Arm ESP32 code
 * 
 * @section intro_sed Introduction
 * 
 * This esp-idf code is used to control my personal robotic arm 
 * with the ESP32 devkitC.
 * 
 * Built automatically: https://travis-ci.org/bmleedy/utiliArm
 * GitHub README: https://github.com/bmleedy/utiliArm  
 * 
 * This code is a work in progress.
 *
 * @section dependencies Dependencies
 * 
 * - ESP IDF - https://dl.espressif.com/doc/esp-idf/latest/get-started/linux-setup.html 
 * - ESP32Servo - https://github.com/ShellAddicted/ESP32Servo 
 * 
 * @section author Author
 * 
 * Written by Brett "bmleedy" Leedy for fun and profit.
 * 
 * Copyright 2018 Brett M Leedy
 * 
 * @section license License
 * 
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// Components from this project
#include "ServoAxis.h"
#include "StepperAxis.h"
#include "webserver.h"
#include "SharedKeyStore.h"

// Stepper constants
#define STEPPER_STEP_PIN GPIO_NUM_17       ///< controller step input
#define STEPPER_DIRECTION_PIN GPIO_NUM_22  ///< controller direction input
#define STEPPER_LIMIT_PIN GPIO_NUM_23     ///< todo: bogus
#define STEP_PERIOD_MS 25   ///< step every n milliseconds
#define STEPS_TO_SWEEP 400  ///< go this many steps, then switch directions

// Servo Constants
#define SERVO_MIN_ANGLE       0  ///< minimum angle servos can command
#define SERVO_MAX_ANGLE     180  ///< maximum angle servos can command
#define SERVO_INITIAL_ANGLE  90  ///< go to this angle when whe init
#define SERVO_NUM_AXES        6  ///< number of axes to initialize


struct ServoConfig {
    gpio_num_t pin;
    uint8_t starting_angle;
};

/*! @var servo_output_pins
 *   this array holds the pins we use for servo outputs */
static const ServoConfig servo_output_pins[] = {
    {GPIO_NUM_14,   33},  // D Axis
    {GPIO_NUM_27,   90},  // NC
    {GPIO_NUM_32,   90},  // NC
    {GPIO_NUM_33,  170},  // C Axis
    {GPIO_NUM_25,   25},  // F Axis
    {GPIO_NUM_26,   50}   // E Axis
};



/*! @var axes
 *   array of pointers to generic robot axes to be passed
 *   to callbacks which control the axes. */
RobotAxis * axes[SERVO_NUM_AXES];

/*! @var s_wifi_event_group
 * FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

/* @var WIFI_CONNECTED_BIT
 * event group bit for wifi connect */
const int WIFI_CONNECTED_BIT = BIT0;

/* @var TAG
 * tag for this app's log entries */
static const char *TAG = "wifi station";

/* @var s_retry_num
 * how many times have we retried connecting to wifi?*/
static int s_retry_num = 0;

/*!
 * @fn event_handler
 * 
 * @brief special function to handle freeRTOS system events
 * 
 * The switch statement in this function takes action when a system 
 * event (like wifi connect or disconnect) occurs.
 */
static esp_err_t event_handler(void *ctx, system_event_t *event) {
  switch (event->event_id) {
  case SYSTEM_EVENT_STA_START:
    esp_wifi_connect();
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    ESP_LOGI(TAG, "got ip:%s",
        ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED: {
    if (s_retry_num < 5) {
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    }
    ESP_LOGI(TAG, "connect to the AP fail\n");
    break;
  }
  default:
    break;
  }
  return ESP_OK;
}

/*!
 * @fn initialize_wifi
 * 
 * @brief Set up the wifi adaptor config
 * 
 * The intialize_wifi function handles all of the initialization
 * needed to set the ESP32 up as a wifi station.
 */
void initialize_wifi() {
  ESP_LOGD(TAG, "initialize_wifi(): initializing.");
  s_wifi_event_group = xEventGroupCreate();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  // create station config
wifi_config_t  sta_config;
  memset(&sta_config, 0, sizeof(sta_config));
  strncpy(reinterpret_cast<char *>(sta_config.sta.ssid), CONFIG_ESP_WIFI_SSID,
      32);
  strncpy(reinterpret_cast<char *>(sta_config.sta.password),
      CONFIG_ESP_WIFI_PASSWORD, 32);
  sta_config.sta.bssid_set = false;

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_connect() );
  ESP_LOGD(TAG, "initializew_wifi(): connect to ap SSID:%s password:%s",
      reinterpret_cast<char *>(sta_config.sta.ssid),
      reinterpret_cast<char *>(sta_config.sta.password));
}

/*!
 * @fn app_main
 * 
 * @brief main app that spawns all application processes
 * 
 * This is the root application which calls all configuration
 * and spawns all processes.
 */
#define INCREMENT 2
#define AXIS_DELAY_PERIOD 50

#define STEPPER_TEST_ENA GPIO_NUM_18
#define STEPPER_TEST_DIR GPIO_NUM_5
#define STEPPER_TEST_STP GPIO_NUM_18

extern "C" void app_main() {

  gpio_pad_select_gpio(GPIO_NUM_23);
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_INPUT);


  SharedKeyStore * key_store = new SharedKeyStore(100);

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES
      || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Start WIFI
  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  initialize_wifi();

  int32_t array_len = sizeof(servo_output_pins)/sizeof(servo_output_pins[0]);
  for (int i = 0; i < array_len; i++) {
    ESP_LOGI(TAG, "initializing servo axis %d", i);
    axes[i] = new ServoAxis(SERVO_MAX_ANGLE,
    SERVO_MIN_ANGLE,
    servo_output_pins[i].starting_angle, servo_output_pins[i].pin, i);
  }

/*
  // todo: new servo test code
  int32_t increment_value = INCREMENT;
  int32_t position = 90; //degrees
  while(1) {
    ESP_LOGI(TAG, "sending %d to axis 0, GPIO: %d", position,gpio_get_level(GPIO_NUM_16));
    axes[0]->go_to(position);

    position = position + increment_value;

    if(position > 180 || position < 0) {
      increment_value = -1 * increment_value;
      position = position + increment_value;
    }
    vTaskDelay( AXIS_DELAY_PERIOD );

  }
*/

/*
  // todo: stepper test code
  gpio_pad_select_gpio(STEPPER_TEST_ENA);
  gpio_pad_select_gpio(STEPPER_TEST_DIR);
  gpio_pad_select_gpio(STEPPER_TEST_STP);

  gpio_set_direction(STEPPER_TEST_ENA, GPIO_MODE_OUTPUT);
  gpio_set_direction(STEPPER_TEST_DIR, GPIO_MODE_OUTPUT);
  gpio_set_direction(STEPPER_TEST_STP, GPIO_MODE_OUTPUT);

  gpio_set_level(STEPPER_TEST_ENA,0);
  gpio_set_level(STEPPER_TEST_DIR,1);
  gpio_set_level(STEPPER_TEST_STP,1);

  bool direction_state = false;

  while (gpio_get_level(GPIO_NUM_23)) {
    gpio_set_level(STEPPER_TEST_DIR, direction_state);

    for(int i=0; i<1600; i++){
      vTaskDelay(1);
      gpio_set_level(STEPPER_TEST_STP, 1);
      vTaskDelay(1);
      gpio_set_level(STEPPER_TEST_STP, 0);
      if(gpio_get_level(GPIO_NUM_23) != 1)
        break;
    }
    direction_state = !direction_state;
    ESP_LOGI(TAG, "setting direction to %d", direction_state);
  }
  gpio_set_level(STEPPER_TEST_ENA, 1);
*/
/*
  gpio_pad_select_gpio(STEPPER_TEST_ENA);
  gpio_set_direction(STEPPER_TEST_ENA, GPIO_MODE_OUTPUT);
  gpio_set_level(STEPPER_TEST_ENA,1);

  gpio_pad_select_gpio(STEPPER_DIRECTION_PIN);
  gpio_set_direction(STEPPER_DIRECTION_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(STEPPER_DIRECTION_PIN,0);

  //gpio_pad_select_gpio(STEPPER_TEST_ENA);
  //gpio_set_direction(STEPPER_TEST_ENA, GPIO_MODE_OUTPUT);
  //gpio_set_level(STEPPER_TEST_ENA,0);

  StepperAxis * test_stepper = new StepperAxis(180, 0, 90, 1600,
      STEPPER_STEP_PIN, STEPPER_DIRECTION_PIN, STEPPER_LIMIT_PIN, 0);
  while (1){

    vTaskDelay(10000);
    ESP_LOGI(TAG, "moving to 80");
    test_stepper->go_to(80);
    vTaskDelay(10000);
    ESP_LOGI(TAG, "moving to 100");
    test_stepper->go_to(135);
    vTaskDelay(10000);
  }
  */
  // Start the webserver, whose callbacks move the axes
  start_webserver(axes, SERVO_NUM_AXES);
}
