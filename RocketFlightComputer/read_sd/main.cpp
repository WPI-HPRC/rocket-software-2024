#include <cstdio>
#include <cstdlib>
#include <cstdint>

#pragma pack(push,1)
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
    float magX = 0.0f;
    float magY = 0.0f;
    float magZ = 0.0f;
    float pressure = 0.0f;

    uint32_t servoPosition = 0;

    // Calculated Values
    float altitude = 0.0f;

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
    uint32_t epochTime = 0;
    uint8_t satellites = 0;
    bool gpsLock = false;

    uint32_t loopCount = 0;
    uint32_t timestamp = 0;

};
#pragma pack(pop)

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Provide a file to read data from\n");
    return -1;
  }
  FILE * file = fopen(argv[1], "r");
  if (file == NULL) {
    fprintf(stderr, "failed to open file %s\n", argv[1]);
    return -1;
  }

  uint8_t buf[sizeof(struct TelemPacket)];
  struct TelemPacket packet;

  while (fread(buf, sizeof(struct TelemPacket), 1, file)) {
    packet = *(struct TelemPacket *)buf;
    printf( "%hhd,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%hhd,%hhd,%d,%d\n",
          packet.state, packet.accelX, packet.accelY, packet.accelZ,
          packet.gyroX, packet.gyroY, packet.gyroZ,
          packet.magX, packet.magY, packet.magZ,
          packet.pressure, packet.altitude,
          packet.w, packet.i, packet.j, packet.k,
          packet.posX, packet.posY, packet.posZ,
          packet.velX, packet.velY, packet.velZ,
          packet.gpsLat, packet.gpsLong, packet.gpsAltMSL, packet.gpsAltAGL,
          packet.epochTime, packet.satellites, packet.gpsLock,
          packet.loopCount, packet.timestamp);
  }
  return 0;
}
