#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "SoftwareSerial.h"
#include <queue.h>
#include <semphr.h>
#include <Player.hpp>

SoftwareSerial mp3(11, 10);
int interruptPin = 2;
static SemaphoreHandle_t semaphore; 

void TaskPlay(void *pvParameters);
void TaskReadUart(void *pvParameters);
void blinkInterrupt();


void setup()
{
  semaphore = xSemaphoreCreateBinary();
  Serial.begin(9600);
  xTaskCreate(TaskPlay, "taskPlay", 128, NULL, 1, NULL);
  xTaskCreate(TaskReadUart, "taskReadUart", 128, NULL, 1, NULL);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blinkInterrupt, FALLING);
  vTaskStartScheduler();
}

void loop()
{
}

void TaskPlay(void *pvParameters)
{
  mp3.begin(9600);
  vTaskDelay( 500 / portTICK_PERIOD_MS );
  command(0x0C, 0, mp3); // reset  
  vTaskDelay( 500 / portTICK_PERIOD_MS ); // Wait for initialization
  command(0x09, 0x0002, mp3); // SD card as source
  vTaskDelay( 200 / portTICK_PERIOD_MS ); //  Wait for initialization
  command(0x06, 30, mp3); // Set volume to 30
  play(2, 10, mp3);
  while(1)
  {  
    xSemaphoreTake(semaphore, portMAX_DELAY);
    Serial.println("I will open music 1");
    play(1, 10, mp3);
    Serial.println("Waiting for music");
  }
}

void TaskReadUart(void *pvParameters)
{
  while(1)
  { 
    Serial.print(Serial.available());
    Serial.println(" bytes from uart to read");
    
    if (Serial.available() > 9)
    {
      uint8_t incomingBytes[10];
      for(uint8_t step=0; step<10;step++)
          incomingBytes[step] = Serial.read();

      Serial.print("I received: ");
      Serial.println(incomingBytes[4], HEX);
      Serial.print(" and ");
      Serial.println(incomingBytes[7], HEX);
    }
    vTaskDelay( 10000 / portTICK_PERIOD_MS );
  }
}

void blinkInterrupt()
{
    Serial.println("Sending to play");
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    constexpr long minimalTimeBetweenInterrupts = 1000;
    if (interrupt_time - last_interrupt_time > minimalTimeBetweenInterrupts) 
    {
        xSemaphoreGive(semaphore);
    }
    last_interrupt_time = interrupt_time;
}
