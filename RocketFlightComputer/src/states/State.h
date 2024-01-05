#pragma once
#define _STATE_CLASS_IMPLS_          \
private:                             \
	void initialize_impl() override; \
	void loop_impl() override;       \
	State *nextState_impl() override;

#include "Arduino.h"
#include "../utility.hpp"
#include <ArduinoEigen.h>
/**
 * @brief Abstract class representing a rocket state.
 */
class State {
	public:
		/**
		 * @brief Code to be run once when the state begins.
		 */
		void initialize();
		/**
		 * @brief Code to be run every iteration of the loop while the rocket is in this state.
		 */
		void loop();
		/**
		 * @brief Code run every iteration of the loop to determine the next state to transition to.
		 * @return The pointer to the next state or nullptr if the state has not changed.
		 */
		State *nextState();
		virtual ~State() {}

		struct SensorPacket {
			// Raw Sensor Readings
			float accelX;
			float accelY;
			float accelZ;
			float gyroX;
			float gyroY;
			float gyroZ;
			uint32_t magX;
			uint32_t magY;
			uint32_t magZ;
			float pressure;

			// Calculated Values
			float altitude;

			// State Estimator Outputs
			float q;
			float i;
			float j;
			float k;

			// GPS Inputs
			float gpsLat;
			float gpsLong;
			float gpsAltMSL;
			float gpsAltAGL;

			long timestamp;
    	};	
		SensorPacket sensorPacket;

		Eigen::Vector<float, 10> x_state;

	protected:
		//! @brief number of milliseconds since the initialize call
		long long currentTime = 0;
		//! @brief number of milliseconds since the last loop call
		long long deltaTime = 0;
		//! @brief loop count since initialization
		long long loopCount = 0;

	private:
		//! @brief number of milliseconds from boot to the initialize call
		long long startTime = 0;
		//! @brief number of milliseconds from boot to the previous loop call
		long long lastLoopTime = 0;
		virtual void initialize_impl() = 0;
		virtual void loop_impl() = 0;
		virtual State *nextState_impl() = 0;
};
