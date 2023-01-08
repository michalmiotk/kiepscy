#pragma once

#include "SoftwareSerial.h"
#include <semphr.h>

struct GlobalVariables{
  SoftwareSerial& softwareSerial;
  SemaphoreHandle_t& semaphore;
};