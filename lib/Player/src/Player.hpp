#pragma once

#include "SoftwareSerial.h"

void command(int8_t cmd, int16_t dat, SoftwareSerial& mp3);
void play(uint8_t mp3name, int seconds, SoftwareSerial& mp3, uint8_t dirName=1);