#include "mbed.h"
#include "Motor.h"

Motor::Motor(PinName EAD_in,
             PinName EBD_in
             )
    : ead(EAD_in), ebd(EBD_in)//, // pwm pin names
      //timer(&Motor::wrap_execute_control, osTimerPeriodic, this)//,
      //controller(1,1,0,MOTOR_PERIOD_SEC) // PID controller.  Can't set P=0 initially, apparently.
{
    ead.period_ms(3); //~300Hz, as specified by HW engineer.
    ebd.period_ms(3);

    pwm_val = 0;

    /*
    //changed from 600...
    controller.setInputLimits(0,25); //25 m-sec, theoretical highest I've seen.  Need to perform real assessment on max value
    controller.setOutputLimits(0,1); //PWM can go from 0 to full on, to full on to zero, without braking, that is.  Therefore, -1 to 1
    controller.setMode(1);  //turn PID ON.
    controller.setBias(0);
    controller.setInterval(MOTOR_PERIOD_SEC);
    */
    is_inverted = false;

    /*
    timer.start(MOTOR_PERIOD);

    enc = en;
    */

    pwm_speed(0.0);
}


/*
void Motor::set_setpoint(float setSpeed)
{
    //setPoint = setSpeed;
    controller.setSetPoint(setSpeed);
}

float Motor::get_setpoint()
{
    return controller.getSetpoint();
}

void Motor::wrap_execute_control(const void *motor)
{
    const_cast<Motor*>(static_cast<const Motor*>(motor))->execute_control();
}

void Motor::execute_control()  //function to execute closed loop control
{
    // Only change the speed if the motor is running in auto
    // (speed-controlled) mode
    if (is_auto) 
    {
        
        controller.setProcessValue(enc_speed());
        float val = controller.compute();
        //pc->printf("%.4f PWM \n\r",val);
        //pc->printf("%.2f process_val \n\r",enc_speed());
        //pc->printf("%.2f set \n\r",controller.getSetpoint());
        //pwm_speed(clamp(pwm_val + val,0.0,1.0));
        pwm_speed(clamp(val,0.0,1.0));
    }
}
*/


void Motor::pwm_speed(float value)
{
    float val;
    pwm_val = value;
    if(is_inverted)
    {
        val = -value;
    }
    else
    {
        val = value;
    }

    if(val == (float) 0.0f)
    {
        //write_mosfets(0,0,0,0);
        ead.write(0.0);
        ebd.write(0.0);
    }
    else if(val > 0.0f) //let forward mean that it's AH,BL
    {
        ead.write(val);
        ebd.write(0.0);
    }
    else//let backward mean that it's Al,BH
    {
        ead.write(0.0);
        ebd.write(-val);
        //negative because value is less than 1 here.
    }
}
/*
void Motor::write_mosfets(float valAH,float valAL,float valBH,float valBL) //helper fxn
{
        ah.write(valAH);
        al.write(valAL);
        bh.write(valBH);
        bl.write(valBL);
}
*/
void Motor::brake(float value) //throw on the breaks!  Short motor across GND.
{
    //write_mosfets(0,value,0,value);

}

/*
float Motor::current() //returns current, in amps.  Signed, for +/- current
{
    /*
    float val = ai.read() - .7592;//.7592 is the zero point observed on some tests (converted into scale between 0-1, not perfectly reliable.  May need individual start-up zero offset...
    //val is now centered about zero
    val = val*3.3/.185; //times 3.3, since 3.3V max sense.  /.185 since 185mv/A
    return val;
    */
    //return 0; //THIS CODE DOESN"T WORK
//}
/*
float Motor::enc_speed()//return's encoder's Speed!
{

    return enc->speed();  //should be ., except -> b/c 1 encoder.

}

void Motor::set_auto(bool val)
{
    is_auto = val;
    if(val)
    {
        controller.setMode(1);
    }
    else
    {
        controller.setMode(0);
        //zero is manual
    }
}

*/
float Motor::clamp(float input, float min, float max)
{
    if( input > max)
    {
        return max;
    }
    else if (input < min)
    {
        return min;
    }
    else
    {
        return input;
    }

}
