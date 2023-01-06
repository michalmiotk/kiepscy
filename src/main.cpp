#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "SoftwareSerial.h"
#include <queue.h>
 

SoftwareSerial mp3(11, 10);
QueueHandle_t xQueue;


void command(int8_t cmd, int16_t dat)
{
  vTaskDelay( 20 / portTICK_PERIOD_MS );
  uint8_t cmdbuf[8] = {0};
  cmdbuf[0] = 0x7e; // bajt startu
  cmdbuf[1] = 0xFF; // wersja
  cmdbuf[2] = 0x06; // liczba bajtow polecenia
  cmdbuf[3] = cmd;  // polecenie
  cmdbuf[4] = 0x00; // 0x00 = no feedback, 0x01 = feedback
  cmdbuf[5] = (int8_t)(dat >> 8); // parametr DAT1
  cmdbuf[6] = (int8_t)(dat); //  parametr DAT2
  cmdbuf[7] = 0xef; // bajt konczacy
 
  for (uint8_t i = 0; i < 8; i++)
  {
    mp3.write(cmdbuf[i]);
  }
}
 

inline void play(uint8_t mp3name, int seconds, uint8_t dirName)
{
  uint16_t numberOfDirAndMp3 = dirName<<8;
  Serial.println("obliczylem numer folderu");
  Serial.println(numberOfDirAndMp3);
  numberOfDirAndMp3 += mp3name;
  command(0x0F, numberOfDirAndMp3);
  vTaskDelay(seconds*1000 / portTICK_PERIOD_MS); 
  command(0x0e, 0x00);
}

void TaskPlay( void *pvParameters );
void blink(){
    Serial.println("Sending to play");
    uint8_t val = 1;
    xQueueSendToBackFromISR(xQueue,&val,pdFALSE);
}
int interruptPin = 2;

void setup() {

  xQueue = xQueueCreate( 10, sizeof( uint8_t ) );

  /* We want this queue to be viewable in a RTOS kernel aware debugger,
  so register it. */
  vQueueAddToRegistry( xQueue, "NumberOfSong" );
  Serial.begin(9600);
 

  xTaskCreate(
    TaskPlay
    ,  "taskPlay"
    ,  128  
    ,  NULL
    ,  1  
    ,  NULL);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, FALLING);
  vTaskStartScheduler();
}

void loop()
{ 
}


void TaskPlay(void *pvParameters)  {
  mp3.begin(9600);
  vTaskDelay( 500 / portTICK_PERIOD_MS ); 
  while(1)
  { 
    uint8_t received_number_of_song;
    if(xQueueReceive(xQueue, (void*)&received_number_of_song, 2000 / portTICK_PERIOD_MS)){
        Serial.println("I will open music");
        Serial.println(received_number_of_song);
        play(received_number_of_song, 10, (uint8_t)1);
    }
    Serial.println("Waiting for music");
  }
}