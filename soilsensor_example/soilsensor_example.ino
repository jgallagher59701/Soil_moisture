
#include "Adafruit_seesaw.h"
#include "LowPower.h"

#define SOIL_SENSOR_POWER 8

Adafruit_seesaw ss;
LowPowerClass lp;

void setup() {
	Serial.begin(9600);

	Serial.println("seesaw Soil Sensor example!");

	// Set up the soil sensor
	pinMode(SOIL_SENSOR_POWER, OUTPUT);
	digitalWrite(SOIL_SENSOR_POWER, HIGH);

	if (!ss.begin(0x36)) {
		Serial.println("ERROR! seesaw not found");
		while (1)
			;
	} else {
		Serial.print("seesaw started! version: ");
		Serial.println(ss.getVersion(), HEX);
	}
}

int count = 0;
int wait[] = {1000, 2000, 4000, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
const int wait_size = 12;

void loop() {
	// Turn on the soil sensor and read a value to reset capacitive sensor
	digitalWrite(SOIL_SENSOR_POWER, HIGH);
	uint16_t capread = ss.touchRead(0);
	Serial.print("Capacitive after wake up: ");
	Serial.print(capread);
	Serial.print(", delay: ");
	Serial.println(wait[count]);

	// A 2s wait seems needed, although the begin() method only uses 0.5s
	delay(wait[count++]);
	if (count > wait_size)
		count = 0;

	float tempC = ss.getTemp();
	capread = ss.touchRead(0);

	Serial.print("Temperature: ");
	Serial.print(tempC);
	Serial.println("C");
	Serial.print("Capacitive: ");
	Serial.println(capread);
	Serial.flush();		// flush() i/o before sleep

	delay(4000);	// Here to make reading power easy

	// Turn off the sensor. Drops current to the sensor to 0.20 mA from 4.8 mA
	digitalWrite(SOIL_SENSOR_POWER, LOW);

	// 'Turn off' the I2C bus. Doing this drops current to the sensor to 0.00 mA.
	// without it, current reads 0.20 mA
	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);

	// Sleep for 8s
	lp.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
}
