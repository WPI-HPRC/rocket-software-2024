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
    void handleInterrupt();
  private:
    SFE_MMC5983MA mag;

    volatile bool newDataAvailable = true;
    uint32_t rawValX = 0;
    uint32_t rawValY = 0;
    uint32_t rawValZ = 0;
  
    double scaledX = 0.0;
    double scaledY = 0.0;
    double scaledZ = 0.0;
};
