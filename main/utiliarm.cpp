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
#define STEPPER_STEP_PIN GPIO_NUM_23       ///< controller step input
#define STEPPER_DIRECTION_PIN GPIO_NUM_22  ///< controller direction input
#define STEPPER_LIMIT_PIN GPIO_NUM_25     ///< todo: bogus
#define STEP_PERIOD_MS 25   ///< step every n milliseconds
#define STEPS_TO_SWEEP 400  ///< go this many steps, then switch directions

// Servo Constants
#define SERVO_MIN_ANGLE       0  ///< minimum angle servos can command
#define SERVO_MAX_ANGLE     180  ///< maximum angle servos can command
#define SERVO_INITIAL_ANGLE  90  ///< go to this angle when whe init
#define SERVO_NUM_AXES        6  ///< number of axes to initialize

/*! @var servo_output_pins
 *   this array holds the pins we use for servo outputs */
static const gpio_num_t servo_output_pins[SERVO_NUM_AXES] = { GPIO_NUM_14,
    GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26 };

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
extern "C" void app_main() {
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

  for (int i = 0; i < 6; i++) {
    ESP_LOGI(TAG, "initializing servo axis %d", i);
    axes[i] = new ServoAxis(SERVO_MAX_ANGLE,
    SERVO_MIN_ANGLE,
    SERVO_INITIAL_ANGLE, servo_output_pins[i], i);
  }

  // Start the webserver, whose callbacks move the axes
  start_webserver(axes, SERVO_NUM_AXES);
}
