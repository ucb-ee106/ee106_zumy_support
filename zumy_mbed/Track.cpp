#include "mbed.h"
#include "Track.h"


	Track::Track(PinName motor_1, PinName motor_2, PinName enc_A, PinName enc_B, int pulses_per_rev):
		motor(motor_1,motor_2),
		enc(enc_A,enc_B,NC,pulses_per_rev)
	 //construct the track object.
	{

		closed_loop = false;
	}

	void Track::manual_speed(float dutyCycle)
	{
		closed_loop = false;
		motor.pwm_speed(dutyCycle);
	}
	void Track::set_velocity_setpoint(float speed)
	{
		closed_loop = true;
	}

	int Track::get_position()
	{
		return enc.getPulses();
	}
	float Track::get_speed()
	{
		return 0.0; //TODO
	}

	void Track::set_gains(float kp, float ki, float kd)
	{

	}

