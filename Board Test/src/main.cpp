#include <Arduino.h>
#include <Wire.h>

#include <SparkFun_u-blox_GNSS_v3.h>

SFE_UBLOX_GNSS myGNSS;

void setup() {
	Serial.begin(9600);

	while(!Serial);
	Serial.println("GNSS Example:");

	Wire.begin();

	if (myGNSS.begin() == false) {
		Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring. Freezing."));
		while (1);
	}

	// myGNSS.setNMEAOutputPort(Serial);
}

double prevLat;
double prevLon;

String getChanged(double lat, double lon) {
	String state;
	if (lat == prevLat && lon == prevLon) {
		state = "false.";
	} else {
		state = "true.";
	}
	prevLat = lat;
	prevLon = lon;
	return state;
}

void loop() {
	double lat = myGNSS.getLatitude();
	double lon = myGNSS.getLongitude();
	Serial.println("Latitude: " + String(lat) + ". Longitude: " + String(lon) + ". Changed: " + getChanged(lat, lon));
	

	delay(250); // Don't pound too hard on the I2C bus
}
