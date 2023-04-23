#pragma once

#include <stdint.h>
#include <string>
#include "bmp280.h"

int8_t BMP280_Init(std::string i2cdev);
bool BMP280_Measure();
double BMP280_GetTemperature();
double BMP280_GetPressure();
