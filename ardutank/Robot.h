#include <Arduino.h>

#include "Servo.h"
#include "HCSR04.h"
#include "TB6612FNG.h"

#define MAX_DISTANCE 400
#define MIN_SPEED 100
#define MAX_SPEED 255
#define MAX_SPEED_CMSEC 45
#define MAX_SPEED_GRADSEC 375
#define DISTANCE_INVALID 0

#define MAX_LEFT_SPEED 249
#define MAX_RIGHT_SPEED 255


#define DIRECTION_INVALID 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_RIGHT 3
#define DIRECTION_LEFT 4

#define MAX_RECORD 100

#define OBSTACLE_DISTANCE 25

// ---------------------------------------------------------------------------
// Robot classe
// ---------------------------------------------------------------------------

class Robot {
	public:
		Robot(byte pin_sx_pwm, byte pin_sx_in2, byte pin_sx_in1, byte pin_dx_pwm, byte pin_dx_in2, byte pin_dx_in1, byte pin_sero, uint8_t pin_sonar_trig, uint8_t pin_sonar_echo);
		float getDistance(int i = 0);
		byte getDirection(int i = 0);
		unsigned int getDistanceRepeat();
		unsigned int getDirectionRepeat();
		unsigned int getObstableRepeat();
		void addDistance(float recordvalue);
		void addDirection(byte recordvalue);
		void move(int distance_cm, byte speed, boolean stop_graduate = false);
		void turn(int angle_grad, byte speed, boolean stop_graduate = false);
		void stop(boolean graduate = false);
		float distance(boolean useforcount = false);
		int sweep_dx(float &max_dist, boolean useforcount = false);
		int sweep_sx(float &max_dist, boolean useforcount = false);
		void angle(byte angle);
	private:
		int sweep(int a1, int a2,float &dist, boolean useforcount = false);
		unsigned int REC_dist_info[2];
		unsigned int REC_dir_info[2];
		float REC_dist[MAX_RECORD];
		byte REC_dir[MAX_RECORD];
		unsigned int obstacle_count;
		HCSR04 *sonar;
		Motor *motor_left;
		Motor *motor_right;
		Servo *servo;
};