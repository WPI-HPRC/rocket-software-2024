#include <cstdint>
#include <cstdio>
#include <cstdlib>

#pragma pack(push, 1)
struct TelemPacket {
        uint8_t packetType = 0x01;
        // State Integer
        // 0 - PreLaunch
        // 1 - Launch
        // 2 - Coast
        // 3 - DrogueDescent
        // 4 - MainDescent
        // 5 - Recovery
        // 6 - Abort
        uint8_t state = 0;
        // Raw Sensor Readings
        float accelX = 0.0f;
        float accelY = 0.0f;
        float accelZ = 0.0f;
        float gyroX = 0.0f;
        float gyroY = 0.0f;
        float gyroZ = 0.0f;
        float rawMagX = 0.0f;
        float rawMagY = 0.0f;
        float rawMagZ = 0.0f;
        float pressure = 0.0f;
        float temperature = 0.0f;

        uint32_t servoPosition = 0;

        // Calculated Values
        float altitude = 0.0f;
        float launchAltitude = 0.0f;
        float magX = 0.0f;
        float magY = 0.0f;
        float magZ = 0.0f;

        // EKF Results
        float w = 0.0f; // Quaternion State
        float i = 0.0f;
        float j = 0.0f;
        float k = 0.0f;
        float posX = 0.0f; // Position State ECEF
        float posY = 0.0f;
        float posZ = 0.0f;
        float velX = 0.0f; // Velocity State ECEF
        float velY = 0.0f;
        float velZ = 0.0f;

        // GPS Inputs
        float gpsLat = 0.0f;
        float gpsLong = 0.0f;
        float gpsAltMSL = 0.0f;
        float gpsAltAGL = 0.0f;
        uint32_t gpsVelN = 0.0f;
        uint32_t gpsVelE = 0.0f;
        uint32_t gpsVelD = 0.0f;
        uint32_t epochTime = 0;
        uint8_t satellites = 0;
        bool gpsLock = false;

        int sdFileNo = -1;

        uint32_t loopCount = 0;
        uint32_t timestamp = 0;
        float covQW = 0.0;
        float covQX = 0.0;
        float covQY = 0.0;
        float covQZ = 0.0;
        bool drogueDeploy = false;
        bool mainDeploy = true;
};
#pragma pack(pop)

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Provide a file to read data from\n");
    return -1;
  }
  FILE *file = fopen(argv[1], "r");
  if (file == NULL) {
    fprintf(stderr, "failed to open file %s\n", argv[1]);
    return -1;
  }

  uint8_t buf[sizeof(struct TelemPacket)];
  struct TelemPacket packet;

  printf("state,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,rawMagX,rawMagY,rawMagZ,pressure,temperature,servoPosition,altitude,launchAltitude,magX,magY,magZ,w,i,j,k,posX,posY,posZ,velX,velY,velZ,gpsLat,gpsLong,gpsAltMSL,gpsAltAGL,epochTime,satellites,gpsLock,loopCount,timestamp,covQW,covQX,covQY,covQZ,drogue,main\n");
  while (fread(buf, sizeof(struct TelemPacket), 1, file)) {
    packet = *(struct TelemPacket *)buf;
    printf("%hhd,", packet.state);
    printf("%f,", packet.accelX);
    printf("%f,", packet.accelY);
    printf("%f,", packet.accelZ);
    printf("%f,", packet.gyroX);
    printf("%f,", packet.gyroY);
    printf("%f,", packet.gyroZ);
    printf("%f,", packet.rawMagX);
    printf("%f,", packet.rawMagY);
    printf("%f,", packet.rawMagZ);
    printf("%f,", packet.pressure);
    printf("%f,", packet.temperature);
    printf("%d,", packet.servoPosition);
    printf("%f,", packet.altitude);
    printf("%f,", packet.launchAltitude);
    printf("%f,", packet.magX);
    printf("%f,", packet.magY);
    printf("%f,", packet.magZ);
    printf("%f,", packet.w);
    printf("%f,", packet.i);
    printf("%f,", packet.j);
    printf("%f,", packet.k);
    printf("%f,", packet.posX);
    printf("%f,", packet.posY);
    printf("%f,", packet.posZ);
    printf("%f,", packet.velX);
    printf("%f,", packet.velY);
    printf("%f,", packet.velZ);
    printf("%f,", packet.gpsLat);
    printf("%f,", packet.gpsLong);
    printf("%f,", packet.gpsAltMSL);
    printf("%f,", packet.gpsAltAGL);
    printf("%d,", packet.epochTime);
    printf("%hhd,", packet.satellites);
    printf("%hhd,", packet.gpsLock);
    printf("%d,", packet.sdFileNo);
    printf("%d,", packet.loopCount);
    printf("%d,", packet.timestamp);
    printf("%f,", packet.covQW);
    printf("%f,", packet.covQX);
    printf("%f,", packet.covQY);
    printf("%f,", packet.covQZ);
    printf("%hhd,", packet.drogueDeploy);
    printf("%hhd\n", packet.mainDeploy);
  }
  return 0;
}
