#ifndef TRACK_H_
#define TRACK_H_

#include "mbed.h"
#include "rtos.h"
#include "QEI.h"
#include "pindefs.h"
#include "Motor.h"
#include "MovingAverage.h"


class Track{

private:


	QEI enc; //the encoder
	Motor motor; //the motor
	Ticker controller;
	bool closed_loop;
    MovingAverage encoder_changes;
    int old_position; //tracks the last encoder position, for use with dTicks
	int inverted; //set to -1 for inverted
	//PID controller; //the PID controller

	void execute_timeout(); //internal control loop



public:

	void manual_speed(float dutyCycle); //enable, and take, direct manual control over the motor
	void set_velocity_setpoint(float speed); //enabled the closed-loop controller, and set the velocity setpoint of the controller

	int get_position(); //return the position, in encoder ticks, of the track
	float get_speed(); //return the speed, in ticks per second, of the track.  Averaged over the last... ?? seconds/data.  Need to tune this, but it'll likely be fast.

	void set_gains(float kp, float ki, float kd); //set the gains of the PID velocity controller.  

	Track(PinName motor_1, PinName motor_2, PinName enc_A, PinName enc_B, int pulses_per_rev); //construct the track object.

	void invert(bool invert_state); //invert the direction for both the encoder and the motor.  The encoder and motor are internally self-consistant due to construction

};


#endif
