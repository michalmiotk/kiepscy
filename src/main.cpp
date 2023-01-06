#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "SoftwareSerial.h"
#include <queue.h>
#include <semphr.h>

SoftwareSerial mp3(11, 10);
static SemaphoreHandle_t semaphore;


void command(int8_t cmd, int16_t dat)
{
  vTaskDelay( 20 / portTICK_PERIOD_MS );
  uint8_t cmdbuf[8];
  cmdbuf[0] = 0x7e; // bajt startu
  cmdbuf[1] = 0xFF; // wersja
  cmdbuf[2] = 0x06; // liczba bajtow polecenia
  cmdbuf[3] = cmd;  // polecenie
  cmdbuf[4] = 0x01; // 0x00 = no feedback, 0x01 = feedback
  cmdbuf[5] = (int8_t)(dat >> 8); // parametr DAT1
  cmdbuf[6] = (int8_t)(dat); //  parametr DAT2
  cmdbuf[7] = 0xef; // bajt konczacy
 
  for (uint8_t i = 0; i < 8; i++)
  {
    mp3.write(cmdbuf[i]);
  }
}
 

void play(uint8_t mp3name, int seconds, uint8_t dirName=1)
{
  uint16_t numberOfDirAndMp3 = dirName<<8;
  numberOfDirAndMp3 += mp3name;
  command(0x0F, numberOfDirAndMp3);
  vTaskDelay(seconds*1000 / portTICK_PERIOD_MS); 
  command(0x0e, 0x00);
}

void TaskPlay(void *pvParameters);
void TaskReadUart(void *pvParameters);

void blink(){
    
    Serial.println("Sending to play");
    
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 1000) 
    {
        xSemaphoreGive(semaphore);
    }
    last_interrupt_time = interrupt_time;
}
int interruptPin = 2;

void setup() {
  semaphore = xSemaphoreCreateBinary();
  Serial.begin(9600);
  xTaskCreate(TaskPlay, "taskPlay", 128, NULL, 1, NULL);
  xTaskCreate(TaskReadUart, "taskReadUart", 128, NULL, 1, NULL);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, FALLING);
  vTaskStartScheduler();
}

void loop()
{}

void TaskPlay(void *pvParameters)  {
  mp3.begin(9600);
  vTaskDelay( 500 / portTICK_PERIOD_MS );
  command(0x0C, 0); // reset układu
  vTaskDelay( 500 / portTICK_PERIOD_MS ); // Czekamy 500ms na inicjalizacje  
  command(0x09, 0x0002); // Wybieramy karte SD jako zrodlo
  vTaskDelay( 200 / portTICK_PERIOD_MS ); // Czekamu 200ms na inicjalizacje
  command(0x06, 0x001E); // Ustaw glosnosc na 30
  play(2, 10);
  while(1)
  {  
    xSemaphoreTake(semaphore, portMAX_DELAY);
    Serial.println("I will open music 1");
    play(1, 10);
    Serial.println("Waiting for music");
  }
}

void TaskReadUart(void *pvParameters){
  while(1)
  { 
    Serial.print("dostepne");
    Serial.println(Serial.available());
    if (Serial.available() > 9) {
      // read the incoming byte:
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