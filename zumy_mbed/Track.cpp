#include "mbed.h"
#include "Track.h"

int avg_width = 10; //average over 10 cycles of the encoders.  Still, i'll do control on dTicks, not on the boxcar averaged data.
//low speeds: 400 ticks/sec.  
float encoder_read_rate = .002; //sec


Track::Track(PinName motor_1, PinName motor_2, PinName enc_A, PinName enc_B, int pulses_per_rev):
	motor(motor_1,motor_2),
	enc(enc_A,enc_B,NC,pulses_per_rev),
	encoder_changes(avg_width,0) //initial dTicks = 0
	//construct the track object.
{

	closed_loop = false;
    controller.attach(this, &Track::execute_timeout,encoder_read_rate);
    old_position = 0; //I start at encoder equals zero.


    inverted = 1;
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
	return inverted*enc.getPulses();
}
float Track::get_speed()
{
	return encoder_changes.GetAverage()/encoder_read_rate; //divide by encoder read rate to recover ticks/sec.
}

void Track::set_gains(float kp, float ki, float kd)
{

}

void Track::execute_timeout()
{
	//called every 2 ms.  open question if it should be an RTOS timer.
	int pos = inverted*enc.getPulses();
	encoder_changes.Insert(pos-old_position);
	old_position = pos;
}

void Track::invert(bool invert_state)
{
	motor.is_inverted = invert_state;
	if(invert_state) //i AM inverted
	{
		inverted = -1;
	}
	else  //i am NOT inverted
	{
		inverted = 1;
	}
}