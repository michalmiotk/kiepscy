#include <Tasks.hpp>
#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include "Player.hpp"
#include "GlobalVariables.hpp"

void TaskPlay(void *pvParameters)
{
  if(pvParameters == nullptr){
    return;
  }
  GlobalVariables* globalVariables = (GlobalVariables*)pvParameters;
  setupPlayer(globalVariables->softwareSerial);
  play(2, 10, globalVariables->softwareSerial);
  while(1)
  {  
    xSemaphoreTake(globalVariables->semaphore, portMAX_DELAY);
    Serial.println("I will open music 1");
    uint8_t mp3Number = (millis() % 2) + 1;
    play(mp3Number, 60, globalVariables->softwareSerial);
    Serial.println("Waiting for music");
    xSemaphoreTake(globalVariables->semaphore, 20 / portTICK_PERIOD_MS);
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

      Serial.print("Received from soft uart: ");
      Serial.println(incomingBytes[4], HEX);
      Serial.print(" and ");
      Serial.println(incomingBytes[7], HEX);
    }
    vTaskDelay( 20000 / portTICK_PERIOD_MS );
  }
}
