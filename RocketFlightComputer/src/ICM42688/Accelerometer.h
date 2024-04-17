#pragma once
#include <ICM42688.h>

typedef struct {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
} ICM_data;

class Accelerometer {
  public:
    Accelerometer(int addr);
    bool init();
    ICM_data read();
  private:
    ICM42688 icm42688;
};
