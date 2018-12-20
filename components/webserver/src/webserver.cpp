#include "webserver.h"

#define WEBSERVER_LOG_TAG "webserver.cpp"

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

/* Our URI handler function to be called during GET /uri request */
esp_err_t get_handler(httpd_req_t *req) {
  /* Send a simple response */
  httpd_resp_send(req, main_page_raw.c_str(), main_page_raw.length());
  return ESP_OK;
}

/* Our URI handler function to be called during POST /uri request */
esp_err_t post_handler(httpd_req_t *req) {
  /* Destination buffer for content of HTTP POST request.
   * httpd_req_recv() accepts char* only, but content could
   * as well be any binary data (needs type casting).
   * In case of string data, null termination will be absent, and
   * content length would give length of string */
  char content[100];

  /* Truncate if content length larger than the buffer */
  size_t recv_size = MIN(req->content_len, sizeof(content));

  int ret = httpd_req_recv(req, content, recv_size);
  if (ret <= 0) { /* 0 return value indicates connection closed */
    /* Check if timeout occurred */
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      /* In case of timeout one can choose to retry calling
       * httpd_req_recv(), but to keep it simple, here we
       * respond with an HTTP 408 (Request Timeout) error */
      httpd_resp_send_408(req);
    }
    /* In case of error, returning ESP_FAIL will
     * ensure that the underlying socket is closed */
    return ESP_FAIL;
  }

  /* Send a simple response */
  const char * resp = "URI POST Response";
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

/* Our URI handler function to be called during POST /uri request */
esp_err_t axis_post_handler(httpd_req_t *req) {
  // todo: pass size with this context
  RobotAxis ** axes = reinterpret_cast<RobotAxis **>(req->user_ctx);

  /* Destination buffer for content of HTTP POST request.
   * httpd_req_recv() accepts char* only, but content could
   * as well be any binary data (needs type casting).
   * In case of string data, null termination will be absent, and
   * content length would give length of string */
  char content[101];

  /* Truncate if content length larger than the buffer */
  // Always leave one character in the buffer for the null termination
  size_t recv_size = MIN(req->content_len, (sizeof(content) - 1));

  int ret = httpd_req_recv(req, content, recv_size);

  // Null terminate the content after receiving it for printing
  content[recv_size] = '\0';

  if (ret <= 0) { /* 0 return value indicates connection closed */
    /* Check if timeout occurred */
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      /* In case of timeout one can choose to retry calling
       * httpd_req_recv(), but to keep it simple, here we
       * respond with an HTTP 408 (Request Timeout) error */
      httpd_resp_send_408(req);
    }
    /* In case of error, returning ESP_FAIL will
     * ensure that the underlying socket is closed */
    return ESP_FAIL;
  }

  // print the body data
  ESP_LOGI(WEBSERVER_LOG_TAG, "POST Request Data: %s", content);

  // todo: parse the body data and fill these vars
  // scan through string, finding all the keys and putting them in a map
  std::map < std::string, std::string > request_data_map;
  std::string request_data_string;
  request_data_string.assign(content, recv_size);

  // todo: turn this code into an actual parsing function
  int32_t start_position = 0;  // always start at first char
  int32_t equal_index = 0;       // index of equal sign in key-value pair
  int32_t and_index = 0;         // index of and sign terminating key-value pair
  while (start_position < (request_data_string.size() - 1)) {
    // find the index of the end of the key
    equal_index = request_data_string.find('=', start_position);
    if (equal_index == -1)
      break;  // no '=' found. No keys to add
    // find the index of the end of the value
    and_index = request_data_string.find('&', (equal_index + 1));
    if (and_index == -1)  // no '&' found.  rest of string is value
      and_index = request_data_string.size();
    /*
     ESP_LOGD(WEBSERVER_LOG_TAG, "received substrings at [%d,%d,%d]  %s : %s",
     start_position,equal_index,and_index,
     request_data_string.substr(start_position,(equal_index-start_position)).c_str(),
     request_data_string.substr(equal_index+1,(and_index-equal_index-1)).c_str() );
     */
    request_data_map[request_data_string.substr(start_position,
        (equal_index - start_position))] = request_data_string.substr(
        equal_index + 1, (and_index - equal_index - 1));

    ESP_LOGD(WEBSERVER_LOG_TAG, "received key-value pair [%d]  %s : %s",
        start_position,
        request_data_string.substr(start_position,
            (equal_index - start_position)).c_str(),
        request_data_map[request_data_string.substr(start_position,
            (equal_index - start_position))].c_str());

    start_position = and_index + 1;  // last value should be out of range
  }

  // interpret the keys we need
  int axis_number = atoi(request_data_map["axis"].c_str());
  int axis_value = atoi(request_data_map["value"].c_str());
  // todo: parsing internet input is dangerous - do lots of checking here
  // skip if we didn't get the parts we need

  // todo: move an axis here  todo: do something with the return value
  ESP_LOGI(WEBSERVER_LOG_TAG, "moving axis index %d to position %d",
      axis_number, axis_value);
  axes[axis_number]->go_to(axis_value);

  /* Send a simple response */
  const char * resp = "URI POST Response";
  httpd_resp_send(req, resp, strlen(resp));
  return ESP_OK;
}

/* URI handler structure for GET /uri */
httpd_uri_t uri_get = { .uri = "/", .method = HTTP_GET, .handler = get_handler,
    .user_ctx = NULL };

/* URI handler structure for POST /uri */
httpd_uri_t uri_post = { .uri = "/controls", .method = HTTP_POST, .handler =
    axis_post_handler, .user_ctx = NULL };

/* Function for starting the webserver */
httpd_handle_t start_webserver(RobotAxis ** axes, int32_t num_axes) {
  /* Generate default configuration */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK) {
    /* Register URI handlers */
    uri_get.user_ctx = reinterpret_cast<void *>(axes);
    httpd_register_uri_handler(server, &uri_get);
    uri_post.user_ctx = reinterpret_cast<void *>(axes);
    httpd_register_uri_handler(server, &uri_post);
  }
  /* If server failed to start, handle will be NULL */
  return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server) {
  if (server) {
    /* Stop the httpd server */
    httpd_stop(server);
  }
}
