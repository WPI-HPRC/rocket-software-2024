#pragma once
#include <SparkFun_MMC5983MA_Arduino_Library.h>

class Magnetometer {
  public:
    Magnetometer();
    bool init();
  private:
    SFE_MMC5983MA mag;
};
