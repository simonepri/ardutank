#include <Arduino.h>

#define MAX_SENSOR_DISTANCE 500
#define US_ROUNDTRIP_CM 57

#define NO_ECHO 0
#define MAX_SENSOR_DELAY 18000
#define ECHO_TIMER_FREQ 24
#define PING_MEDIAN_DELAY 29

#define HCSR04Convert(echoTime, conversionFactor) (echoTime ? float((echoTime + float(conversionFactor / 2.0)) / float(conversionFactor)) : 0)


class HCSR04 {
	public:
		HCSR04(uint8_t trigger_pin, uint8_t echo_pin, int max_cm_distance = MAX_SENSOR_DISTANCE);
		unsigned int ping();
		float ping_cm();
		unsigned int ping_median(uint8_t it = 5);
		float convert_cm(unsigned int echoTime);
		void ping_timer(void (*userFunc)(void));
		boolean check_timer();
		unsigned long ping_result;
		static void timer_us(unsigned int frequency, void (*userFunc)(void));
		static void timer_ms(unsigned long frequency, void (*userFunc)(void));
		static void timer_stop();
	private:
		boolean ping_trigger();
		boolean ping_wait_timer();
		uint8_t _triggerBit;
		uint8_t _echoBit;
		volatile uint8_t *_triggerOutput;
		volatile uint8_t *_triggerMode;
		volatile uint8_t *_echoInput;
		unsigned int _maxEchoTime;
		unsigned long _max_time;
		static void timer_setup();
		static void timer_ms_cntdwn();
};