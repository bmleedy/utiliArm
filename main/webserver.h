#ifndef UTILIARMWEBSRV_H
#define UTILIARMWEBSRV_H

#include <esp_http_server.h>

void stop_webserver(httpd_handle_t server);
httpd_handle_t start_webserver(void);


#endif
