#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "SoftwareSerial.h"
#include "MyDFRobotDFPlayerMini.hpp"

#include <SoftwareSerial.h>
 
SoftwareSerial mp3(11, 10);
 
static uint8_t cmdbuf[8] = {0};
 
void command(int8_t cmd, int16_t dat)
{
  vTaskDelay( 20 / portTICK_PERIOD_MS );
 
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
 

void volumeProxy(uint8_t volume){

}

void TaskBlink1( void *pvParameters );

void TaskBlink2( void *pvParameters );

void Taskprint( void *pvParameters );

void setup() {
  Serial.begin(9600);
    
  xTaskCreate(

    TaskBlink1

    ,  "task1"   

    ,  128  

    ,  NULL

    ,  1  

    ,  NULL );

  xTaskCreate(

    TaskBlink2

    ,  "task2"

    ,  128  

    ,  NULL

    ,  1  

    ,  NULL );

    xTaskCreate(

    Taskprint

    ,  "task3"

    ,  128  

    ,  NULL

    ,  1  

    ,  NULL );
vTaskStartScheduler();

}

void loop()

{

}

void TaskBlink1(void *pvParameters)  {
  
  mp3.begin(9600);
  vTaskDelay( 500 / portTICK_PERIOD_MS );  // Czekamu 200ms na inicjalizacje
  vTaskDelay( 200 / portTICK_PERIOD_MS );  // Czekamu 200ms na inicjalizacje
 
  command(0x0F, 0x0101);
  vTaskDelay( 10000 / portTICK_PERIOD_MS ); 
  command(0x0e, 0x00);
  command(0x0F, 0x0102);
  vTaskDelay( 10000 / portTICK_PERIOD_MS ); 
  command(0x0e, 0x00);

  pinMode(8, OUTPUT);

  while(1)

  {

    Serial.println("Task1");

    digitalWrite(8, HIGH);   

    vTaskDelay( 200 / portTICK_PERIOD_MS ); 

    digitalWrite(8, LOW);    

    vTaskDelay( 200 / portTICK_PERIOD_MS ); 

  }

}

void TaskBlink2(void *pvParameters)  

{
  const int ledPin = 13;
  pinMode(ledPin, OUTPUT);

  while(1)

  {

    Serial.println("Task2");

    digitalWrite(ledPin, HIGH);   

    vTaskDelay( 300 / portTICK_PERIOD_MS ); 

    digitalWrite(ledPin, LOW);   

    vTaskDelay( 300 / portTICK_PERIOD_MS ); 

  }

}

void Taskprint(void *pvParameters)  {

  int counter = 0;

  while(1)

  {

counter++;

  Serial.println(counter); 

  vTaskDelay(500 / portTICK_PERIOD_MS);    }

}