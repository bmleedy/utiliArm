#include "SharedKeyStore.h"

/*
 uint16_t max_key_len_;
 uint16_t max_keys_;
 std::map< std::string, std::string> key_store_;
 SemaphoreHandle_t write_mutex_;  //this is a binary semaphore
 SemaphoreHandle_t read_mutex_;   //this is a counting semaphore
 */

// todo: create a test program that just puts some values in
//   from 2 threads and takes some out
// todo: I'm implementing the key_store as std::map of strings,
//  which I'm a bit concerned is going to blow up my heap when
//  I update the value strings.  If this actually happens, I
//  will re-implement as pre-allocated array of strings with
//  a vector of name strings as an index.
SharedKeyStore::SharedKeyStore(uint16_t max_keys) {
  max_keys_ = max_keys;
  write_mutex_ = xSemaphoreCreateBinary();
}

bool SharedKeyStore::insert_or_update(std::string key, std::string value) {
  bool rv = false;

  if (has_key(key)) {
    // The key exists, update it
    if (xSemaphoreTake(write_mutex_, 10)) {   // take semaphore
      // todo: I don't think I need to worry about deadlock,
      //  since my priority gets bumped by anyone trying to
      //  take my lock
      //  if I do need to worry about deadlock, then disble interrupts here
      vTaskSuspendAll();  // do not switch off of me (does not suspend
                          // interrupts)
      key_store_[key].assign(value);
      xSemaphoreGive(write_mutex_);
      xTaskResumeAll();  // restart scheduler
      rv = true;  // write success
    } else {
      ESP_LOGE(KEYSTORE_TAG, "failed to write key %s", key.c_str());
      rv = false;  // failed to write
    }
  } else if (key_store_.size() <= max_keys_) {
    // there's room, add a new key
    if (xSemaphoreTake(write_mutex_, 10)) {   // take semaphore
      // todo: I don't think I need to worry about deadlock,
      //  since my priority gets bumped by anyone trying to
      //  take my lock
      //  if I do need to worry about deadlock, then disble interrupts here
      vTaskSuspendAll();  // do not switch off of me (does not suspend
                          // interrupts)
      key_store_[key] == value;
      xSemaphoreGive(write_mutex_);
      xTaskResumeAll();  // restart scheduler
      rv = true;  // write success
    } else {
      ESP_LOGE(KEYSTORE_TAG, "failed to add key %s (could not take semaphore)",
          key.c_str());
      rv = false;  // failed to write
    }

  } else {
    // not enough room, print an error and return false
    ESP_LOGE(KEYSTORE_TAG, "failed to add key %s because keystore is full",
        key.c_str());
  }

  return rv;
}

bool SharedKeyStore::get(std::string key, std::string * value) {
  if (has_key(key)) {
    value->assign(key_store_[key]);
  } else {
    ESP_LOGE(KEYSTORE_TAG, "failed to get key [%s]. Key not in keystore.",
        key.c_str());
  }
  return true;
}

// todo: doxygen

bool SharedKeyStore::has_key(std::string key) {
  return key_store_.find(key) != key_store_.end();
}

