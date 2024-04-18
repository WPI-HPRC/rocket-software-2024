#include "State.h"
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

    apogeeEstimator = new ApogeeEstimation();
    xbee->start();

/*

    const uint16_t networkID = 0x4843;

    uint8_t byte_array[2];
    
    break_uint16(networkID, byte_array);

    xbee->setParameter(XBee::AtCommand::ApiOptions, 0x90);
    xbee->setParameter(AsciiToUint16('I', 'D'), byte_array, 2);
*/

    // exit(0);
}

void State::loop() {
	long now = millis();
	this->currentTime = now - this->startTime;
	this->deltaTime = now - this->lastLoopTime;
	this->loopCount++;
	this->sensorPacket = this->sensors->readSensors();
  this->telemPacket.altitude = Utility::pressureToAltitude(this->sensorPacket.pressure);
	if (this->stateEstimatorInitialized) {
        this->stateEstimator->onLoop(this->sensorPacket);

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

        

        Serial.print("QUAT|"); Serial.print(this->stateEstimator->x(0)); Serial.print(",");
        Serial.print(this->stateEstimator->x(1)); Serial.print(",");
        Serial.print(this->stateEstimator->x(2)); Serial.print(",");
        Serial.println(this->stateEstimator->x(3));

        // float hdg = atan2(sensorPafcket.magY, sensorPacket.magX) * (180/PI);
        /*
        let angle = Math.atan2(y, x);
angle = angle * (180 / Math.PI)
angle = angle + 90
angle = (angle +360) % 360*/


        this->telemPacket.w = this->stateEstimator->x(0);
        this->telemPacket.i = this->stateEstimator->x(1);
        this->telemPacket.j = this->stateEstimator->x(2);
        this->telemPacket.k = this->stateEstimator->x(3);
    }

    loop_impl();
    this->lastLoopTime = now;

    /**
     * Assemble Telemetry packet from sensor packet, this is stuff we want done every loop
     */
    this->telemPacket.state = this->getId();
    this->telemPacket.accelX = this->sensorPacket.accelX;
    this->telemPacket.accelY = this->sensorPacket.accelY;
    this->telemPacket.accelZ = this->sensorPacket.accelZ;

    this->telemPacket.gyroX = this->sensorPacket.gyroX;
    this->telemPacket.gyroY = this->sensorPacket.gyroY;
    this->telemPacket.gyroZ = this->sensorPacket.gyroZ;

    this->telemPacket.magX = this->sensorPacket.magX;
    this->telemPacket.magY = this->sensorPacket.magY;
    this->telemPacket.magZ = this->sensorPacket.magZ;

    this->telemPacket.pressure = this->sensorPacket.pressure;

    this->telemPacket.gpsLat = this->sensorPacket.gpsLat;
    this->telemPacket.gpsLong = this->sensorPacket.gpsLong;
    this->telemPacket.epochTime = this->sensorPacket.epochTime;
    this->telemPacket.satellites = this->sensorPacket.satellites;
    this->telemPacket.gpsLock = this->sensorPacket.gpsLock;
    this->telemPacket.loopCount = this->loopCount;
    this->telemPacket.timestamp = now;

#ifdef SERIAL_TELEMETRY
    this->telemPacket.debugPrint();
#endif



    /** Loop Radio and Send Data */

    // Serial.printf("Loop count: %llu\n", this->loopCount);

#ifndef NO_XBEE

    xbee->sendTransmitRequestCommand(0x0013A200423F474C, (uint8_t *)&telemPacket, sizeof(telemPacket));

    // Serial.print("Packet Success: ");
    // Serial.println(now);
#endif
}

State *State::nextState()
{
#ifndef NO_TRANSITION
    return nextState_impl();
#else
		return nullptr;
#endif
}
