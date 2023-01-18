#include "Player.hpp"
#include <Arduino_FreeRTOS.h>

void command(int8_t cmd, int16_t dat, SoftwareSerial& mp3)
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

void play(uint8_t mp3name, int seconds, SoftwareSerial& mp3, uint8_t dirName=1)
{
  uint16_t numberOfDirAndMp3 = dirName<<8;
  numberOfDirAndMp3 += mp3name;
  command(0x0F, numberOfDirAndMp3, mp3);
  vTaskDelay(seconds*1000 / portTICK_PERIOD_MS); 
  command(0x0e, 0x00, mp3);
}

void setupPlayer(SoftwareSerial& softwareSerial)
{
    vTaskDelay( 500 / portTICK_PERIOD_MS );
    command(0x0C, 0,  softwareSerial); // reset  
    vTaskDelay( 500 / portTICK_PERIOD_MS ); // Wait for initialization
    command(0x09, 0x0002, softwareSerial); // SD card as source
    vTaskDelay( 200 / portTICK_PERIOD_MS ); //  Wait for initialization
    command(0x06, 30, softwareSerial); // Set volume to 30
}