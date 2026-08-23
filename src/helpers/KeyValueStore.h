#pragma once
#include <stdint.h>
#include <string.h>

class KeyValueStore {
protected:
  KeyValueStore() { }
public:
  virtual void setByKey(const char* key, const char* value) { }
  virtual void getByKey(const char* key, char* value, size_t max_len) { }
};
