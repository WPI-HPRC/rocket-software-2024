#pragma once
#include <Arduino.h>

constexpr static uint8_t X_OUT_0 = 0x00;

constexpr static uint8_t X_OUT_1 = 0x01;

constexpr static uint8_t Y_OUT_0 = 0x02;

constexpr static uint8_t Y_OUT_1 = 0x03;

constexpr static uint8_t Z_OUT_0 = 0x04; 

constexpr static uint8_t Z_OUT_1 = 0x05;

constexpr static uint8_t XYZ_OUT_2 = 0x06;

constexpr static uint8_t T_OUT = 0x07; // Temperature Register

constexpr static uint8_t STATUS = 0x08; // MMC Status

constexpr static uint8_t INT_CTRL_0 = 0x09; // Control Register 0

constexpr static uint8_t INT_CTRL_1 = 0x0A; // Control Register 1

constexpr static uint8_t INT_CTRL_2 = 0x0B; // Control Register 2

constexpr static uint8_t INT_CTRL_3 = 0x0C; // Control Register 3

constexpr static uint8_t PROD_ID = 0x2F;

constexpr static uint8_t MMC_PROD_ID = 0x30; // Sensor Product ID