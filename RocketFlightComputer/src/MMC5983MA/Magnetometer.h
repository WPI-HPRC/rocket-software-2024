#pragma once
#include <SparkFun_MMC5983MA_Arduino_Library.h>

typedef struct {
  uint32_t x;
  uint32_t y;
  uint32_t z;
} MMC_data;

class Magnetometer {
  public:
    Magnetometer();
    bool init();
    MMC_data read();
  private:
    SFE_MMC5983MA mag;
};
