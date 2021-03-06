#ifndef COMPONENTS_WEBSERVER_SRC_WEBSERVER_H_
#define COMPONENTS_WEBSERVER_SRC_WEBSERVER_H_


#include <string.h>
#include <string>
#include <iostream>
#include <map>
#include <memory>
#include "esp_http_server.h"

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
#include "main_page.html.hh"
#include "RobotAxis.h"


void stop_webserver(httpd_handle_t server);
httpd_handle_t start_webserver(RobotAxis ** axes, int32_t num_axes);

#endif  // COMPONENTS_WEBSERVER_SRC_WEBSERVER_H_
