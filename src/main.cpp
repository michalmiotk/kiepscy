#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "SoftwareSerial.h"
#include <queue.h>
#include <semphr.h>
#include "Player.hpp"
#include "Tasks.hpp"
#include "GlobalVariables.hpp"

SoftwareSerial mp3(11, 10);
int interruptPin = 2;
static SemaphoreHandle_t semaphore; 
GlobalVariables globalVariables{mp3, semaphore};

void irGateInterrupt();

void setup()
{
  semaphore = xSemaphoreCreateBinary();
  Serial.begin(9600);
  mp3.begin(9600);
  xTaskCreate(TaskPlay, "taskPlay", 128, &globalVariables, 1, NULL);
  xTaskCreate(TaskReadUart, "taskReadUart", 128, NULL, 1, NULL);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), irGateInterrupt, FALLING);
  vTaskStartScheduler();
}

void loop()
{
}

void irGateInterrupt()
{
    Serial.println("Sending to play");
    BaseType_t pxHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(semaphore, &pxHigherPriorityTaskWoken);
}