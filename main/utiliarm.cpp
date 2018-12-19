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

//! @todo adjust ESP stack size for child task.
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


#include "ServoAxis.h"
#include "StepperAxis.h"

#include "webserver.h"
#include "SharedKeyStore.h"

// Stepper test constants
#define STEPPER_STEP_PIN GPIO_NUM_23       ///< controller step input
#define STEPPER_DIRECTION_PIN GPIO_NUM_22  ///< controller direction input
#define STEPPER_LIMIT_PIN GPIO_NUM_25     ///< todo: bogus
#define STEP_PERIOD_MS 25   ///< step every n milliseconds
#define STEPS_TO_SWEEP 400  ///< go this many steps, then switch directions

// Servo test constants
#define SERVO_OUTPUT_PIN GPIO_NUM_19  ///< connected to servo pulse input
#define SERVO_SWEEP_DEGREES 100       ///< full width of servo sweep
#define SERVO_OFFSET 90               ///< location of servo sweep center

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
  ESP_LOGI(TAG, "initialize_wifi(): tcpip_adapter_init...\n");
  s_wifi_event_group = xEventGroupCreate();
  tcpip_adapter_init();

  ESP_LOGI(TAG, "initialize_wifi(): esp_event_loop_init...\n");
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  ESP_LOGI(TAG, "initialize_wifi(): wifi init\n");
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_LOGI(TAG, "initialize_wifi(): wifi set mode\n");
  ESP_ERROR_CHECK (esp_wifi_set_mode(WIFI_MODE_STA) );

  // create station config
wifi_config_t  sta_config;
  memset(&sta_config, 0, sizeof(sta_config));
  strncpy(reinterpret_cast<char *>(sta_config.sta.ssid), CONFIG_ESP_WIFI_SSID,
      32);
  strncpy(reinterpret_cast<char *>(sta_config.sta.password),
      CONFIG_ESP_WIFI_PASSWORD, 32);
  sta_config.sta.bssid_set = false;

  ESP_LOGI(TAG, "initialize_wifi(): wifi set config\n");
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

  ESP_LOGI(TAG, "initialize_wifi(): wifi start\n");
  ESP_ERROR_CHECK (esp_wifi_start() );

ESP_LOGI  (TAG, "wifi connect\n");
  ESP_ERROR_CHECK (esp_wifi_connect() );

ESP_LOGI  (TAG, "initializew_wifi(): connect to ap SSID:%s password:%s",
      reinterpret_cast<char *>(sta_config.sta.ssid),
      reinterpret_cast<char *>(sta_config.sta.password));
}

/*!
 * @fn motion_test_task
 * 
 * @brief performs GPIO operations to sweep motors and servos
 * 
 * This function is spawned as a task to perform GPIO operations to make
 * the servos sweep back and forth.
 */
void motion_test_task(void *pvParameter) {
  int32_t hwm = 0;

  // Configure the GPIO pin for the servo
  ESP_LOGI(TAG, "motion_test_task: creating servocontrol class...");
  ServoAxis * myServo = new ServoAxis(180,0,90,SERVO_OUTPUT_PIN);


  StepperAxis * myStepper = new StepperAxis(180,0,90,1600,STEPPER_STEP_PIN, STEPPER_DIRECTION_PIN,STEPPER_LIMIT_PIN, true);
  myStepper->force_ready();
  
  // initialize state for the step pin and the direction
  bool direction_state = false;
  int servo_command = SERVO_SWEEP_DEGREES / 2;  // degrees


 
  while (1) {

    // Command the servo
    myServo->go_to(servo_command + SERVO_OFFSET);
    myStepper->go_to(servo_command + SERVO_OFFSET);
    
    // Reverse direction state
    direction_state = !direction_state;
    servo_command = -servo_command;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Reversing Servo Direction........%d\n", servo_command);
    hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Motion Task stack high water mark: %d (32-bit words)", hwm);
  }
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
    ESP_ERROR_CHECK (nvs_flash_erase());ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Start WIFI
  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  initialize_wifi();

  // Start the webserver
  start_webserver();

  ESP_LOGI(TAG, "Creating motor control test task. %p", &motion_test_task);
  xTaskCreate(&motion_test_task, "motion_test", 2000, NULL, 5, NULL);
}
