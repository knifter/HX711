#include <Arduino.h>
#include <HX711.h>

void HX711::begin()
{
	pinMode(_sck, OUTPUT);
	pinMode(_dout, INPUT);

	set_gain(_gain);
}

bool HX711::is_ready()
{
	return digitalRead(_dout) == LOW;
}

void HX711::set_gain(gain_t gain)
{
    _gain = gain;
	digitalWrite(_sck, LOW);
	read();
}

long HX711::read()
{
	// wait for the chip to become ready
	while (!is_ready())
    {
		// Will do nothing on Arduino but prevent resets of ESP8266 (Watchdog Issue)
		yield();
	}

	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	// pulse the clock pin 24 times to read the data
	data[2] = shiftIn(_dout, _sck, MSBFIRST);
	data[1] = shiftIn(_dout, _sck, MSBFIRST);
	data[0] = shiftIn(_dout, _sck, MSBFIRST);

	// set the channel and the gain factor for the next reading using the clock pin
	for (unsigned int i = 0; i < _gain; i++)
    {
		digitalWrite(_sck, HIGH);
		digitalWrite(_sck, LOW);
	}

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80)
    {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
			| static_cast<unsigned long>(data[2]) << 16
			| static_cast<unsigned long>(data[1]) << 8
			| static_cast<unsigned long>(data[0]) );

	return static_cast<long>(value);
}

long HX711::read_average(uint8_t times)
{
	long sum = 0;
	for (uint8_t i = 0; i < times; i++)
    {
		sum += read();
		yield();
	}
	return sum / times;
}

double HX711::get_value(uint8_t times)
{
	return read_average(times) - _offset;
}

float HX711::get_units(uint8_t times)
{
	return (read_average(times) - _offset) / _scalefactor;
}

void HX711::tare(uint8_t times) {
	_offset = read_average(times);
}

void HX711::set_scale(float scale)
{
	_scalefactor = scale;
}

float HX711::get_scale()
{
	return _scalefactor;
}

void HX711::set_offset(long offset)
{
	_offset = offset;
}

long HX711::get_offset() {
	return _offset;
}

void HX711::power_down()
{
	digitalWrite(_sck, LOW);
	digitalWrite(_sck, HIGH);
}

void HX711::power_up()
{
	digitalWrite(_sck, LOW);
}
