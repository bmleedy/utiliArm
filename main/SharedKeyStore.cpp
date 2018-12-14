#include "SharedKeyStore.cpp"

//ref: https://thispointer.com/stdmap-tutorial-part-1-usage-detail-with-examples/
//ref: https://www.codeproject.com/Articles/1106491/Sharing-Objects-Between-Threads-in-Cplusplus-the-S
//critical section reference: https://www.freertos.org/FreeRTOS_Support_Forum_Archive/March_2015/freertos_Data_share_between_tasks_-_beginner_1d8a7f60j.html
// FreeRTOS semaphore docs: https://www.freertos.org/a00113.html

class SharedKeyStore {

public:
  //constructor
  SharedKeyStore::SharedKeyStore(uint16_t max_key_len, uint16_t max_keys) {}
  bool insert_key(std::string key, std::string value) {}
  bool get_key(std::string key, &std::string value, uint16_t max_value_len) {}
  bool has_key(std::string key) {}
  
private:
  uint16_t max_key_len_;
  uint16_t max_keys_;
  std::map< std::string, std::string> key_store_;
  SemaphoreHandle_t write_mutex_;  //this is a binary semaphore
  SemaphoreHandle_y read_mutex_;   //this is a counting semaphore
  // read_mutex // can only take when write is open
  // write_mutex //can only take when read and write are open


}



//todo: create a test program that just puts some values in from 2 threads and takes some out
