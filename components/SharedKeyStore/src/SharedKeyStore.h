#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <map>
#include <string>

#define KEYSTORE_TAG "SharedKeyStore"

//todo: cpplint
//todo: doxygen

class SharedKeyStore {

public:
  //constructor
  SharedKeyStore(uint16_t max_keys);
  bool insert_or_update(std::string key, std::string value);
  bool get(std::string key, std::string * value);
  bool has_key(std::string key);

private:
  uint16_t max_key_len_;
  uint16_t max_keys_;
  std::map<std::string, std::string> key_store_;
  SemaphoreHandle_t write_mutex_;  //this is a binary semaphore
  //SemaphoreHandle_t read_mutex_; // todo: not used yet.  optimize to allow concurrent reads

};
