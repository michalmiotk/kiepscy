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
  vTaskDelay( 500 / portTICK_PERIOD_MS );
  command(0x0C, 0, globalVariables->softwareSerial); // reset  
  vTaskDelay( 500 / portTICK_PERIOD_MS ); // Wait for initialization
  command(0x09, 0x0002, globalVariables->softwareSerial); // SD card as source
  vTaskDelay( 200 / portTICK_PERIOD_MS ); //  Wait for initialization
  command(0x06, 30, globalVariables->softwareSerial); // Set volume to 30
  play(2, 10, globalVariables->softwareSerial);
  while(1)
  {  
    xSemaphoreTake(globalVariables->semaphore, portMAX_DELAY);
    Serial.println("I will open music 1");
    uint8_t mp3Number = (millis() % 2) + 1;
    play(mp3Number, 30, globalVariables->softwareSerial);
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
