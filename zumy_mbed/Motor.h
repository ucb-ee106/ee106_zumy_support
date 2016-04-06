#ifndef MOTOR_H_
#define MOTOR_H_

#include "mbed.h"
#include "rtos.h"
#include "QEI.h"
#include "pindefs.h"

class Motor {

private:


    //float setPoint;

	PwmOut ead;  //connecto servo PWM @PTD4	//PwmOut ah;
	PwmOut ebd;
    
    //esgencoder* enc;
    //RtosTimer timer;
    float clamp(float input, float min, float max); //helper clamp functions
    //void execute_control();

    //static void wrap_execute_control(const void *motor);

    

public:
    //bool is_auto;
    float pwm_val;
	void pwm_speed(float dutyCycle); //public only for emergency circumstances, please don't use regularly....
    //void set_setpoint(float setSpeed); //intended speed that you want the motor to go @
    //Motor(PinName A_H, PinName A_L, PinName B_H, PinName B_L,esgencoder* en);
    Motor(PinName EAD_in, PinName EBD_in);
    void brake(float value);
    //float current();
    //float enc_speed(); //returns encoder's speed!
    //float get_setpoint();
    //PID controller;
    //void set_auto(bool val);
    bool is_inverted;  //deep, deep down, reverses motor direction.
};

#endif
