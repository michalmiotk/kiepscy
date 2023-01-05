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
 

void play(uint8_t mp3name, int seconds, uint8_t dirName)
{
  uint16_t numberOfDirAndMp3 = dirName<<8;
  Serial.println("obliczylem numer folderu");
  Serial.println(numberOfDirAndMp3);
  numberOfDirAndMp3 += mp3name;
  command(0x0F, numberOfDirAndMp3);
  vTaskDelay(seconds*1000 / portTICK_PERIOD_MS); 
  command(0x0e, 0x00);
}

void TaskDetect( void *pvParameters );
void TaskPlay( void *pvParameters );

void setup() {
  xQueue = xQueueCreate( 10, sizeof( uint8_t ) );

  /* We want this queue to be viewable in a RTOS kernel aware debugger,
  so register it. */
  vQueueAddToRegistry( xQueue, "NumberOfSong" );
  Serial.begin(9600);
    
  xTaskCreate(
    TaskDetect
    ,  "taskDetect"   
    ,  128  
    ,  NULL
    ,  1  
    ,  NULL );

  xTaskCreate(
    TaskPlay
    ,  "taskPlay"
    ,  128  
    ,  NULL
    ,  1  
    ,  NULL);
vTaskStartScheduler();
}

void loop()
{
}


void TaskDetect(void *pvParameters)  
{
  uint8_t valToSend = 2;
  while(1)
  {

    Serial.println("Send Task");
  
    if(valToSend==1){
      valToSend=2;
    }else{
      valToSend=1;
    }
    xQueueSend( /* The handle of the queue. */
               xQueue,
               ( void * ) &valToSend,
               ( TickType_t ) 0 );
    vTaskDelay( 23000 / portTICK_PERIOD_MS ); 
  }
}

void TaskPlay(void *pvParameters)  {
  mp3.begin(9600);
  vTaskDelay( 500 / portTICK_PERIOD_MS );  // Czekamu 200ms na inicjalizacje
  while(1)
  {
    digitalWrite(8, HIGH);   
    uint8_t received_number_of_song;
    if(xQueueReceive(xQueue, (void*)&received_number_of_song, 500 / portTICK_PERIOD_MS)){
        Serial.println("I will open music");
        Serial.println(received_number_of_song);
        play(received_number_of_song, 10, (uint8_t)1);
    }
    Serial.println("Waiting for music");
  }
}