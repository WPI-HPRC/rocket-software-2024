#pragma once
#include <ICM42688.h>

class Accelerometer {
  public:
    Accelerometer(int addr);
    void init();
  private:
    ICM42688 icm2688;
};
