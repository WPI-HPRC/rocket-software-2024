#include "State.h"
#include "hardware/dma.h"
#include "utility.hpp"
#include <Arduino.h>

State::State(struct Sensors *sensors, StateEstimator *stateEstimator) : sensors(sensors), stateEstimator(stateEstimator) {}

void break_uint16(uint16_t value, uint8_t *byte_array) {
    byte_array[1] = (uint8_t)(value & 0xFF);        // Low byte
    byte_array[0] = (uint8_t)((value >> 8) & 0xFF); // High byte
}

void State::initialize()
{
    this->startTime = millis();
    initialize_impl();


    // const uint16_t networkID = 0x4843;

    // uint8_t byte_array[2];
    
    // break_uint16(networkID, byte_array);

    // xbee.setParameter(XBee::AtCommand::ApiOptions, 0x90);
    // xbee.setParameter(AsciiToUint16('I', 'D'), byte_array, 2);


    // exit(0);
}

void State::loop() {
	uint64_t now = millis();
  // These values may be used in the state code
	this->currentTime = now - this->startTime;
	this->deltaTime = now - this->lastLoopTime;
	this->loopCount++;
  #ifdef PRINT_TIMINGS
  uint64_t start = millis();
  #endif
	this->sensorPacket = this->sensors->readSensors();
  #ifdef PRINT_TIMINGS
  Serial.printf("\tREAD SENSORS TIME: %llu\n", millis() - start);
  #endif
  /**
   * Assemble Telemetry packet from sensor packet, this is stuff we want done every loop
   */
  this->telemPacket.altitude = Utility::pressureToAltitude(this->sensorPacket.pressure);
  this->telemPacket.state = this->getId();
  this->telemPacket.accelX = this->sensorPacket.accelX;
  this->telemPacket.accelY = this->sensorPacket.accelY;
  this->telemPacket.accelZ = this->sensorPacket.accelZ;

  this->telemPacket.gyroX = this->sensorPacket.gyroX;
  this->telemPacket.gyroY = this->sensorPacket.gyroY;
  this->telemPacket.gyroZ = this->sensorPacket.gyroZ;

  this->telemPacket.rawMagX = this->sensorPacket.magX;
  this->telemPacket.rawMagY = this->sensorPacket.magY;
  this->telemPacket.rawMagZ = this->sensorPacket.magZ;

  this->telemPacket.pressure = this->sensorPacket.pressure;

  this->telemPacket.servoPosition = analogRead(SERVO_FEEDBACK_GPIO);

  this->telemPacket.gpsLat = this->sensorPacket.gpsLat;
  this->telemPacket.gpsLong = this->sensorPacket.gpsLong;
  this->telemPacket.epochTime = this->sensorPacket.epochTime;
  this->telemPacket.satellites = this->sensorPacket.satellites;
  this->telemPacket.gpsLock = this->sensorPacket.gpsLock;
  this->telemPacket.loopCount = this->loopCount;
  this->telemPacket.timestamp = now;
  /* Apply Accelerometer Biases */
  //TODO: Make these matrices constants, I tried and it was mad :(
  // BLA::Matrix<3,3> accelScaleFactor = {
  //   1.515094, -0.057311, -0.115285,
  //   -0.057311, 1.081834, 0.021162,
  //   -0.115285, 0.021162, 1.002006
  // };

  // BLA::Matrix<3> accelBias = {-0.043848, 0.071495, 0.018711};

  // BLA::Matrix<3> accelVector = {sensorPacket.accelX, sensorPacket.accelY, sensorPacket.accelZ};

  // BLA::Matrix<3> accelCal = accelScaleFactor * (accelVector - accelBias);

  /* Apply Magnetometer Calibration */

  static const BLA::Matrix<3,3> softIronCal = {
     1.120602,    -0.003242,   0.005510,
    -0.003242,     1.143276,   0.013794,
     0.005510,     0.013794,   1.104641,
  };

  static const BLA::Matrix<3> hardIronCal = {54062.849827, 5545.343210, 89181.770655};

  BLA::Matrix<3> magVector = {sensorPacket.magX, sensorPacket.magY, sensorPacket.magZ};

  BLA::Matrix<3> magCal = softIronCal * (magVector - hardIronCal);

  this->telemPacket.magX = magCal(0);
  this->telemPacket.magY = magCal(1);
  this->telemPacket.magZ = magCal(2);

  // Update sensor packet with the calibrated values
  // this->telemPacket.accelX = accelCal(0);
  // this->telemPacket.accelY = accelCal(1);
  // this->telemPacket.accelZ = accelCal(2);

  // BLA::Matrix<10> testState = {this->stateEstimator->x(0), this->stateEstimator->x(1), this->stateEstimator->x(2), this->stateEstimator->x(3), 0, 0, -305, 0, 0, 0};

  // float apogee = apogeeEstimator->estimate(testState, sensorPacket);

  // Serial.print("[Apogee Estimator] Estimate: "); Serial.println(apogee);

  // Serial.print(sensorPacket.accelX); Serial.print(",");
  // Serial.print(sensorPacket.accelY); Serial.print(",");
  // Serial.print(sensorPacket.accelZ); Serial.print(",");
  // Serial.print(sensorPacket.gyroX); Serial.print(",");
  // Serial.print(sensorPacket.gyroY); Serial.print(",");
  // Serial.print(sensorPacket.gyroZ); Serial.print(",");
  // Serial.print(sensorPacket.magX); Serial.print(",");
  // Serial.print(sensorPacket.magY); Serial.print(",");
  // Serial.print(sensorPacket.magZ); Serial.print(",");
  // Serial.println(sensorPacket.timestamp);

  // double magNorm = sqrt(sensorPacket.magX*sensorPacket.magX + sensorPacket.magY*sensorPacket.magY + sensorPacket.magZ*sensorPacket.magZ);

  // double heading = atan2((sensorPacket.magX / magNorm), 0 - (sensorPacket.magY / magNorm));
  // heading /= PI;
  // heading *= 180;
  // heading += 180;
  // Serial.print("Heading: "); Serial.println(heading);
  // Serial.println(sensorPacket.timestamp);  

	if (this->stateEstimator->initialized) {
    #ifdef PRINT_TIMINGS
    start = millis();
    #endif
		this->stateEstimator->onLoop(this->telemPacket);
    #ifdef PRINT_TIMINGS
    Serial.printf("\tEKF STEP TIME: %llu\n", millis() - start);
    #endif

		this->telemPacket.w = this->stateEstimator->x(0);
		this->telemPacket.i = this->stateEstimator->x(1);
		this->telemPacket.j = this->stateEstimator->x(2);
		this->telemPacket.k = this->stateEstimator->x(3);
	}
	
  #ifdef PRINT_TIMINGS
  start = millis();
  #endif
	loop_impl();
  #ifdef PRINT_TIMINGS
  Serial.printf("\tLOOP IMPL TIME: %llu\n", millis() - start);
  #endif
	this->lastLoopTime = now;

#ifndef NO_SD
  // if (dma_channel_is_busy(sd_spi_dma_chan)) {
  //   Serial.printf("[ERROR] DMA channel still busy, waiting for finish!");
  //   dma_channel_wait_for_finish_blocking(sd_spi_dma_chan);
  // }
#endif

#ifndef NO_XBEE

    #ifdef PRINT_TIMINGS
    start = millis();
    #endif
    SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));
    xbee.sendTransmitRequestCommand(0x0013A200423F474C, (uint8_t *)&telemPacket, sizeof(telemPacket));
    SPI.endTransaction();
    #ifdef PRINT_TIMINGS
    Serial.printf("\tXBEE SEND TIME: %llu\n", millis() - start);
    #endif

    // Serial.print("Packet Success: ");
    // Serial.println(now);
#endif

// #ifndef NO_SD
//   if (sdCardInitialized) {
//     #ifdef PRINT_TIMINGS
//     start = millis();
//     #endif
//     dataFile.write((uint8_t *)&this->telemPacket, sizeof(this->telemPacket));
//     if (this->loopCount % 20 == 0) {
//       dataFile.flush();
//     }
//     #ifdef PRINT_TIMINGS
//     Serial.printf("\tSD WRITE TIME: %llu\n", millis() - start);
//     #endif
//   }
// #endif

#ifdef SERIAL_TELEMETRY
    this->telemPacket.debugPrint();
#endif

  // Serial.print("QUAT|"); Serial.print(this->stateEstimator->x(0)); Serial.print(",");
  // Serial.print(this->stateEstimator->x(1)); Serial.print(",");
  // Serial.print(this->stateEstimator->x(2)); Serial.print(",");
  // Serial.println(this->stateEstimator->x(3));

    /** Loop Radio and Send Data */

    // Serial.printf("Loop count: %llu\n", this->loopCount);
}

State *State::nextState()
{
#ifndef NO_TRANSITION
    return nextState_impl();
#else
		return nullptr;
#endif
}
