#pragma once
#include <SparkFun_MMC5983MA_Arduino_Library.h>

typedef struct {
  double x;
  double y;
  double z;
} MMC_data;

class Magnetometer {
  public:
    Magnetometer();
    bool init();
    MMC_data read();
  private:
    SFE_MMC5983MA mag;
};
