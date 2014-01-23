/*
 * spark_app.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: scottpiette
 *      this app includes all functions to date :
 *
 *      >  reading barometric pressure & temp from the BMP085
 *      >  uptime function to keep track of the current running time
 *      >  reading temperature and humidity from the DHT11
 *      >  flashing an LED for a heart beat
 *
  */

/*
 * Now lets use this part of the source for location of the objects that are used in loop()
 */
#include "application.h"
#include <math.h>

/************************************ spark_iDHT.h **************************/

#define IDDHT11LIB_VERSION "0.1"

// state codes
#define IDDHTLIB_OK						0
#define IDDHTLIB_ACQUIRING				1
#define IDDHTLIB_ACQUIRED				2
#define IDDHTLIB_RESPONSE_OK			3

// error codes
#define IDDHTLIB_ERROR_CHECKSUM			-1
#define IDDHTLIB_ERROR_ISR_TIMEOUT		-2
#define IDDHTLIB_ERROR_RESPONSE_TIMEOUT	-3
#define IDDHTLIB_ERROR_DATA_TIMEOUT		-4
#define IDDHTLIB_ERROR_ACQUIRING		-5
#define IDDHTLIB_ERROR_DELTA			-6
#define IDDHTLIB_ERROR_NOTSTARTED		-7

#define IDDHT11_CHECK_STATE		if(_state == STOPPED)					\
									return _status;						\
								else if(_state != ACQUIRED)				\
									return IDDHTLIB_ERROR_ACQUIRING;

class idDHT11
{
public:
	idDHT11(int pin, int intNumber,void (*isrCallback_wrapper)());
    void init(int pin, int intNumber,void (*isrCallback_wrapper)());
	void isrCallback();
	int acquire();
	int acquireAndWait();
	float getCelsius();
	float getFahrenheit();
	float getKelvin();
	double getDewPoint();
	double getDewPointSlow();
	float getHumidity();
	bool acquiring();
	int getStatus();

private:
	void (*isrCallback_wrapper)(void);

	enum states{RESPONSE=0,DATA=1,ACQUIRED=2,STOPPED=3,ACQUIRING=4};
	volatile states _state;
	volatile int _status;
	volatile uint8_t _bits[6];
	volatile uint8_t _cnt;
	volatile uint8_t _idx;
	volatile int _us;
	int _intNumber;
	int _pin;
	int _type;
	unsigned long _lastreadtime;
	volatile float _hum;
	volatile float _temp;
};

/******************************** spark_iDHT.cpp ************************************/

idDHT11::idDHT11(int pin, int intNumber,void (*callback_wrapper)()) {
	init(pin, intNumber,callback_wrapper);
}

void idDHT11::init(int pin, int intNumber, void (*callback_wrapper) ()) {
	this->_intNumber = intNumber;
	this->_pin = pin;
	this->isrCallback_wrapper = callback_wrapper;
	_hum = 0;
	_temp = 0;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH);
	_state = STOPPED;
	_status = IDDHTLIB_ERROR_NOTSTARTED;
}

int idDHT11::acquire() {
	if (_state == STOPPED || _state == ACQUIRED) {

		/*
		 * Setup the initial state machine
		 */
		_state = RESPONSE;

		/*
		 * Set the initial values in the buffer and variables
		 */
		for (int i=0; i< 6; i++)
			_bits[i] = 0;
		_cnt = 7;
		_idx = 0;
		_hum = 0;
		_temp = 0;

		/*
		 * Toggle the digital output to trigger the DHT device
		 * to send us temperature and humidity data
		 */
		pinMode(_pin, OUTPUT);
		digitalWrite(_pin, LOW);
		delay(18);
		digitalWrite(_pin, HIGH);
		delayMicroseconds(40);
		pinMode(_pin, INPUT);

		/*
		 * Attach the interrupt handler to receive the data once the DHT
		 * starts to send us its data
		 */
		_us = micros();
		attachInterrupt(_intNumber, isrCallback_wrapper, FALLING);

		return IDDHTLIB_ACQUIRING;
	} else
		return IDDHTLIB_ERROR_ACQUIRING;
}

int idDHT11::acquireAndWait() {
	acquire();
	while(acquiring())	;
	return getStatus();
}

void idDHT11::isrCallback() {
	int newUs = micros();
	int delta = (newUs-_us);
	_us = newUs;
	if (delta>6000) {
		_status = IDDHTLIB_ERROR_ISR_TIMEOUT;
		_state = STOPPED;
		detachInterrupt(_intNumber);
		return;
	}
	switch(_state) {
		case RESPONSE:
			if(delta < 25){
				_us -= delta;
				break; //do nothing, it started the response signal
			} if(125 < delta && delta < 190) {
				_state = DATA;
			} else {
				detachInterrupt(_intNumber);
				_status = IDDHTLIB_ERROR_RESPONSE_TIMEOUT;
				_state = STOPPED;
			}
			break;
		case DATA:
			if(delta<10) {
				detachInterrupt(_intNumber);
				_status = IDDHTLIB_ERROR_DELTA;
				_state = STOPPED;
			} else if(60 < delta && delta < 155) { //valid in timing
				if(delta > 90) //is a one
					_bits[_idx] |= (1 << _cnt);
				if (_cnt == 0) { // we have fullfilled the byte, go to next
						_cnt = 7; // restart at MSB
						if(_idx++ == 4) { // go to next byte, if we have got 5 bytes stop.
							detachInterrupt(_intNumber);
							// WRITE TO RIGHT VARS
							// as bits[1] and bits[3] are always zero they are omitted in formulas.
							_hum = _bits[0];
							_temp = _bits[2];
							uint8_t sum = _bits[0] + _bits[2];
							if (_bits[4] != sum) {
								_status = IDDHTLIB_ERROR_CHECKSUM;
								_state = STOPPED;
							} else {
								_status = IDDHTLIB_OK;
								_state = ACQUIRED;
							}
							break;
						}
				} else _cnt--;
			} else {
				detachInterrupt(_intNumber);
				_status = IDDHTLIB_ERROR_DATA_TIMEOUT;
				_state = STOPPED;
			}
			break;
		default:
			break;
	}
}

bool idDHT11::acquiring() {
	if (_state != ACQUIRED && _state != STOPPED)
		return true;
	return false;
}

int idDHT11::getStatus() { return _status; }
float idDHT11::getCelsius() { IDDHT11_CHECK_STATE; return _temp; }
float idDHT11::getHumidity() { IDDHT11_CHECK_STATE; return _hum; }
float idDHT11::getFahrenheit() { IDDHT11_CHECK_STATE; return _temp * 1.8 + 32; }
float idDHT11::getKelvin() { IDDHT11_CHECK_STATE; return _temp + 273.15; }

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double idDHT11::getDewPoint() {
	IDDHT11_CHECK_STATE;
	double a = 17.271;
	double b = 237.7;
	double temp_ = (a * (double) _temp) / (b + (double) _temp) + log( (double) _hum/100);
	double Td = (b * temp_) / (a - temp_);
	return Td;
}
// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
double idDHT11::getDewPointSlow() {
	IDDHT11_CHECK_STATE;
	double a0 = (double) 373.15 / (273.15 + (double) _temp);
	double SUM = (double) -7.90298 * (a0-1.0);
	SUM += 5.02808 * log10(a0);
	SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/a0)))-1) ;
	SUM += 8.1328e-3 * (pow(10,(-3.49149*(a0-1)))-1) ;
	SUM += log10(1013.246);
	double VP = pow(10, SUM-3) * (double) _hum;
	double T = log(VP/0.61078); // temp var
	return (241.88 * T) / (17.558-T);
}

/******************************** spark_bmp.h ************************************/

//#include "Wire.h"

#define BMP085_DEBUG 0

#define BMP085_I2CADDR					0x77

#define BMP085_ULTRALOWPOWER		0
#define BMP085_STANDARD				1
#define BMP085_HIGHRES					2
#define BMP085_ULTRAHIGHRES			3

#define BMP085_CONTROL					0xF4
#define BMP085_TEMPDATA					0xF6
#define BMP085_PRESSUREDATA			0xF6
#define BMP085_READTEMPCMD			0x2E
#define BMP085_READPRESSURECMD	0x34


class spark_BMP {
	public:
		spark_BMP();
		boolean init(uint8_t mode = BMP085_ULTRAHIGHRES);  // by default go highres
		float readTemperature(void);
		int32_t readPressure(void);
		float readAltitude(float sealevelPressure = 101325); // std atmosphere
		uint16_t readRawTemperature(void);
		uint32_t readRawPressure(void);

	private:
		uint8_t read8(uint8_t addr);
		uint16_t read16(uint8_t addr);
		void write8(uint8_t addr, uint8_t data);
		uint8_t oversampling;
		int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
		uint16_t ac4, ac5, ac6;
};

/******************************** spark_bmp.cpp ************************************/

spark_BMP::spark_BMP() {
}

boolean spark_BMP::init(uint8_t mode) {

	if (mode > BMP085_ULTRAHIGHRES)
		mode = BMP085_ULTRAHIGHRES;
	oversampling = mode;

	/*
	 *  There are 22 bytes of calibration data
	 *  Lets read them into the variables
	 */
	Wire.begin();

	/* Not sure what this is about, can't find any reference for this first read in the docs */
	if (read8(0xD0) != 0x55)	return false;

	ac1 = read16(0xAA);
	ac2 = read16(0xAC);
	ac3 = read16(0xAE);
	ac4 = read16(0xB0);
	ac5 = read16(0xB2);
	ac6 = read16(0xB4);
	b1 = read16(0xB6);
	b2 = read16(0xB8);
	mb = read16(0xBA);
	mc = read16(0xBC);
	md = read16(0xBE);

	return true;
}

uint16_t spark_BMP::readRawTemperature(void) {
	write8(BMP085_CONTROL, BMP085_READTEMPCMD);
	delayMicroseconds(4500);
	return read16(BMP085_TEMPDATA);
}

uint32_t spark_BMP::readRawPressure(void) {
	uint32_t raw;

	write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

	if (oversampling == BMP085_ULTRALOWPOWER)
		delayMicroseconds(4500);
	else if (oversampling == BMP085_STANDARD)
		delayMicroseconds(7500);
	else if (oversampling == BMP085_HIGHRES)
		delayMicroseconds(13500);
	else
		delayMicroseconds(25500);

	raw = read16(BMP085_PRESSUREDATA);

	raw <<= 8;
	raw |= read8(BMP085_PRESSUREDATA+2);
	raw >>= (8 - oversampling);

	/*
	 *  this pull broke stuff, look at it later?
	if (oversampling==0) {
    		raw <<= 8;
    		raw |= read8(BMP085_PRESSUREDATA+2);
    		raw >>= (8 - oversampling);
  	  }
	 */

	return raw;
}


int32_t spark_BMP::readPressure(void) {
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;

	UT = readRawTemperature();
	UP = readRawPressure();

	// do temperature calculations
	X1=(UT-(int32_t)(ac6))*((int32_t)(ac5))/pow(2,15);
	X2=((int32_t)mc*pow(2,11))/(X1+(int32_t)md);
	B5=X1 + X2;

	// do pressure calcs
	B6 = B5 - 4000;
	X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
	X2 = ((int32_t)ac2 * B6) >> 11;
	X3 = X1 + X2;
	B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

	X1 = ((int32_t)ac3 * B6) >> 13;
	X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	} else {
		p = (B7 / B4) * 2;
	}

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * p) >> 16;

	p = p + ((X1 + X2 + (int32_t)3791)>>4);

	return p;
}

float spark_BMP::readTemperature(void) {
	int32_t UT, X1, X2, B5;     // following ds convention
	float temp;

	UT = readRawTemperature();

	// step 1
	X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2,15);
	X2 = ((int32_t)mc * pow(2,11)) / (X1+(int32_t)md);
	B5 = X1 + X2;
	temp = (B5+8)/pow(2,4);
	temp /= 10;

	return temp;
}

float spark_BMP::readAltitude(float sealevelPressure) {
	float altitude;

	float pressure = readPressure();

	altitude = 44330 * (1.0 - pow(pressure /sealevelPressure,0.1903));

	return altitude;
}

/*********************************************************************/
uint8_t spark_BMP::read8(uint8_t a) {
	uint8_t ret;

	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.write(a); // sends register address to read from
	Wire.endTransmission(); // end transmission

//	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
	ret = Wire.read(); // receive DATA
//	Wire.endTransmission(); // end transmission

	return ret;
}

uint16_t spark_BMP::read16(uint8_t a) {
	uint16_t ret;

	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.write(a); // sends register address to read from
	Wire.endTransmission(); // end transmission

//	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
	ret = Wire.read(); // receive DATA
	ret <<= 8;
	ret |= Wire.read(); // receive DATA
//	Wire.endTransmission(); // end transmission

	return ret;
}

void spark_BMP::write8(uint8_t a, uint8_t d) {
	Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
	Wire.write(a); // sends register address to read from
	Wire.write(d);  // write data
	Wire.endTransmission(); // end transmission
}

/************************************** END OF OBJECTS *******************************/

/************************************** BEGIN SETUP AND MAIN LOOP *************************/

int heartbeat_led = D7;
bool heartbeat_state = false;

int idDHT11pin = D2; //Digital pin for comunications
int idDHT11intNumber = D2; //interrupt number (must be the one that use the previous defined pin (see table above)
unsigned long _uptime, _lasttime;

//declaration
void dht11_wrapper(); // must be declared before the lib initialization

// DHT instantiate
idDHT11 DHT11(idDHT11pin,idDHT11intNumber,dht11_wrapper);

// BMP085 instaniate
spark_BMP bmp;

void setup()
{
	Serial.begin(9600);
	Serial.println("idDHT11 Example program");
	Serial.print("LIB version: ");
	Serial.println(IDDHT11LIB_VERSION);
	Serial.println("---------------");
	_lasttime = millis();
	_uptime = 0;

	// Initialize the BMP085 sensor
	if (!bmp.init()) {
		Serial.println("Could not find a valid BMP085 sensor, check wiring!");
		while (1) {}
	}

	// Configure the LED heartbeat pin to be output
	pinMode(heartbeat_led, OUTPUT);
	digitalWrite(heartbeat_led, LOW);
	heartbeat_state = false;

}
// This wrapper is in charge of calling
// must be defined like this for the lib work
void dht11_wrapper() {
	DHT11.isrCallback();
}

void loop()
{
	unsigned long _curtime, _days, _hours, _mins, _secs;

	Serial.print("\nRetrieving information from sensor: ");
	Serial.print("\nUpTime: ");
	_curtime = millis();
	_uptime += (_curtime - _lasttime)/1000;
	_lasttime = _curtime;
	_days = _uptime / 86400;
	_hours = (_uptime - (_days * 86400)) / 3600;
	_mins = (_uptime - (_days * 86400 + _hours * 3600)) / 60;
	_secs = (_uptime - (_days * 86400 + _hours * 3600 + _mins * 60));
	Serial.print(_days);
	Serial.print(":");
	Serial.print(_hours);
	Serial.print(":");
	Serial.print(_mins);
	Serial.print(":");
	Serial.println(_secs);
	//delay(100);

	Serial.print("DHT11 sensor: ");
	DHT11.acquire();
	while (DHT11.acquiring())
		;
	int result = DHT11.getStatus();
	switch (result)
	{
		case IDDHTLIB_OK:
			Serial.println("OK");
			break;
		case IDDHTLIB_ERROR_CHECKSUM:
			Serial.println("Error\n\r\tChecksum error");
			break;
		case IDDHTLIB_ERROR_ISR_TIMEOUT:
			Serial.println("Error\n\r\tISR Time out error");
			break;
		case IDDHTLIB_ERROR_RESPONSE_TIMEOUT:
			Serial.println("Error\n\r\tResponse time out error");
			break;
		case IDDHTLIB_ERROR_DATA_TIMEOUT:
			Serial.println("Error\n\r\tData time out error");
			break;
		case IDDHTLIB_ERROR_ACQUIRING:
			Serial.println("Error\n\r\tAcquiring");
			break;
		case IDDHTLIB_ERROR_DELTA:
			Serial.println("Error\n\r\tDelta time to small");
			break;
		case IDDHTLIB_ERROR_NOTSTARTED:
			Serial.println("Error\n\r\tNot started");
			break;
		default:
			Serial.println("Unknown error");
			break;
	}
	Serial.print("Humidity (%): ");
	Serial.println(DHT11.getHumidity(), 2);

	Serial.print("Temperature (oC): ");
	Serial.println(DHT11.getCelsius(), 2);

	Serial.print("Temperature (oF): ");
	Serial.println(DHT11.getFahrenheit(), 2);

/*
	Serial.print("Temperature (K): ");
	Serial.println(DHT11.getKelvin(), 2);

	Serial.print("Dew Point (oC): ");
	Serial.println(DHT11.getDewPoint());
 */

	Serial.print("Dew Point Slow (oC): ");
	Serial.println(DHT11.getDewPointSlow());

	Serial.flush();

	/*
	 * Flash the heartbeat led
	 */
	digitalWrite(heartbeat_led, heartbeat_state?LOW:HIGH);       // Turn LED either on or off
	heartbeat_state = !heartbeat_state;

	/*
	 * Query the readings from the BMP085
	 * Print out the variables
	 */
	Serial.print("BMP085 sensor: \n");
	Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");

    	Serial.flush();

	delay(2000);
}
