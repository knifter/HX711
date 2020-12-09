#include <Arduino.h>
#include <HX711.h>

/**
 *
 * HX711 library for Arduino
 * https://github.com/knifter/lib-HX711
 * Forked from:
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
**/

// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )

#if HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif

// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
uint8_t HX711::shiftin() 
{
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        digitalWrite(_sck, HIGH);
#if FAST_CPU
        delayMicroseconds(1);
#endif
		value |= digitalRead(_dout) << (7 - i);
        digitalWrite(_sck, LOW);
#if FAST_CPU
        delayMicroseconds(1);
#endif
    }
    return value;
};

void HX711::begin()
{
	pinMode(_sck, OUTPUT);
	pinMode(_dout, INPUT);

	set_gain(_gain);
};

bool HX711::is_ready()
{
	return digitalRead(_dout) == LOW;
};

void HX711::set_gain(gain_t gain)
{
    _gain = gain;
	digitalWrite(_sck, LOW);
	read(); // read once because gainfactor is set after read..
};

long HX711::read()
{
	// wait for the chip to become ready
	while (!is_ready())
    {
		// Will do nothing on Arduino but prevent resets of ESP8266 (Watchdog Issue)
		yield();
	};

    union
    {
        uint8_t data[3];
        long value;
    } u;
    
	#if HAS_ATOMIC_BLOCK
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

	#elif IS_FREE_RTOS
	// Begin of critical section.
	// Critical sections are used as a valid protection method
	// against simultaneous access in vanilla FreeRTOS.
	// Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
	// context switches and servicing of ISRs during a critical section.
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

	#else
	// Disable interrupts.
	noInterrupts();
	#endif

	// Pulse the clock pin 24 times to read the data.
	u.data[2] = shiftin();
	u.data[1] = shiftin();
	u.data[0] = shiftin();

	// Create the same delay as shiftIn does between previous and next pulses
	digitalRead(_dout);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < _gain; i++) 
	{
		digitalWrite(_sck, HIGH);
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
		digitalWrite(_sck, LOW);
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
	};

	#if IS_FREE_RTOS
	// End of critical section.
	portEXIT_CRITICAL(&mux);

	#elif HAS_ATOMIC_BLOCK
	}

	#else
	// Enable interrupts again.
	interrupts();
	#endif

    // Sign extend data
	u.data[3] = (u.data[2] & 0x80) ? 0xFF : 0x00;

	// FIXME: code below could probably be just this:
	// return u.value;
	// FIXME: but needs testing first: byte order might be wrong way round

    // Construct a 32-bit signed integer
	uint32_t value = 0;
    value |= static_cast<unsigned long>(u.data[3]) << 24;
	value |= static_cast<unsigned long>(u.data[2]) << 16;
	value |= static_cast<unsigned long>(u.data[1]) << 8;
    value |= static_cast<unsigned long>(u.data[0]);

	return static_cast<long>(value);
};

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) 
{
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready()) 
		{
			return true;
		};
		delay(delay_ms);
	};
	return false;
};

void HX711::wait_ready(unsigned long delay_ms) 
{
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!is_ready()) 
	{
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
		delay(delay_ms);
	};
};

long HX711::read_average(uint8_t times)
{
	long sum = 0;
	for (uint8_t i = 0; i < times; i++)
    {
		sum += read();
		yield();
	};
	return sum / times;
};

double HX711::get_value(uint8_t times)
{
	return read_average(times) - _offset;
};

float HX711::get_units(uint8_t times)
{
	return (read_average(times) - _offset) / _scalefactor;
};

void HX711::tare(uint8_t times) {
	_offset = read_average(times);
};

void HX711::set_scale(float scale)
{
	_scalefactor = scale;
};

float HX711::get_scale()
{
	return _scalefactor;
};

void HX711::set_offset(long offset)
{
	_offset = offset;
};

long HX711::get_offset() {
	return _offset;
};

void HX711::power_down()
{
	digitalWrite(_sck, LOW);
	digitalWrite(_sck, HIGH);
};

void HX711::power_up()
{
	digitalWrite(_sck, LOW);
};