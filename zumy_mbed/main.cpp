#include "mbed.h"
#include "mbed_rpc.h"

#include "QEI.h"
#include "MODSERIAL.h"
#include "rtos.h"
#include "pindefs.h"
#include "Track.h"
#include "imu.h"

//SerialRPCInterface SerialRPC(USBTX, USBRX, 115200);
MODSERIAL pc(USBTX, USBRX); // tx, rx
//Serial pc(USBTX,USBRX);


int r_enc, l_enc;


RPCVariable<int>   rpc_r_enc(&r_enc, "r_enc");
RPCVariable<int>   rpc_l_enc(&l_enc, "l_enc");




DigitalOut init_done(LED1);
DigitalOut main_loop(LED3);
DigitalOut test(LED4);

Track* track_right;
Track* track_left;


//copy_paste from 192.  gets when using carriange returns
int gets_cr(MODSERIAL &src, char *s, int max) {
    int counter = 0;
    char c = 0;
    while(c != '\r')//src.readable() && c != '\r') 
    {
        c = src.getc();
        //pc.printf("%c \n\r",c);
        *(s++) = c;
        counter++;
        if (counter == max-1) break;
    }
    //*(s+1) = ' ';
    *(s++) = '\0';

    return counter;
}


void led_blink_periodic(void const *args) {
    // Toggle the green LED when this function is called.
    test = !test;
}

int main() {


    RtosTimer ledBlinkTimer(led_blink_periodic);
    ledBlinkTimer.start(200);

    char rpc_input_buf[256];
    char rpc_output_buf[1024];
    //copy-pasta from 192, and remove the parts I don't need.

    pc.baud(115200);



    pc.printf("Hello world! \n\r");
    test = 1;
    wait_ms(100);
    test = 0;
    wait_ms(100);

    Track track_left_ = Track(MOTOR_1_1,MOTOR_1_2,p29,p30,624);
    Track track_right_ = Track(MOTOR_2_1,MOTOR_2_2,p11,p12,624); //these both need to be instantiaed inside main, not statically above.
    track_left = &track_left_;
    track_right = &track_right_;
    track_right -> invert(true); //I start inverted.
    pc.printf("Tracks initialized \n\r");

    init_imu();  //also starts an rtos thread for the purpose of reading the IMU.
    pc.printf("IMU is ready is%d \n\r",imu_ready);

    // receive commands, and send back the responses

    while(1) 
    {
        //pc.gets(rpc_input_buf, 256);
        gets_cr(pc,rpc_input_buf,256); //works around the ctrl-enter thing. nneed to append things with a space...
        //pc.printf("input_buf is %sEND \n\r",rpc_input_buf);

        /*

        char* loc = &rpc_input_buf[0];
        pc.printf("Mem addr is %i \n\r",loc);
        for(int i = 0; i<20; i++)
        {
            pc.printf("%i.",*(loc+i));

        }


        */

        //pc.printf("\n\r");
        
        //for(int i = 0; i<20; i++)
        //{
        //    pc.printf("%c.",*(loc+i));
        //}

        pc.printf("\n\r");
        
        //pc.printf("%i..\n\r",rpc_input_buf);
        
        //pc.printf(rpc_input_buf);
        RPC::call(rpc_input_buf, rpc_output_buf);

        pc.printf("%s \n\r>>>", rpc_output_buf);
        Thread::wait(100);

        // Handle the encoders, used for testing if rpc variable reads work
        r_enc=track_right->get_position();
        l_enc=track_left->get_position();

        pc.printf("Effort: %f %f \n\r",track_right->getPwm(),track_left->getPwm());
        pc.printf("Speed: %f %f \n\r",track_right->get_speed(),track_left->get_speed());

    }

}



void sm(Arguments* input, Reply *output);
//Attach it to an RPC object.
RPCFunction rpc_sm(&sm, "sm");
void sm(Arguments* input, Reply *output)
{
    //one argument: near or far.

    //linescan_query_sensor(linescan_buf);

    //copy linescan_buff into new array, so it doesn't get overwritten.

    float arg0 = input->getArg<float>();
    float arg1 = input->getArg<float>();

    //pc.printf("arg0 is %f \n\r",arg0);
    //pc.printf("arg1 is %f \n\r",arg1);
    track_left -> set_auto(true);
    track_right -> set_auto(true);


    track_left->set_velocity_setpoint(arg0);
    track_right->set_velocity_setpoint(arg1);

}

void spd(Arguments* input, Reply *output);
//Attach it to an RPC object.
RPCFunction rpc_spd(&spd, "spd");
void spd(Arguments* input, Reply *output)
{
    //one argument: near or far.

    //linescan_query_sensor(linescan_buf);

    //copy linescan_buff into new array, so it doesn't get overwritten.

    pc.printf("L is %i \n\r",track_left->get_position());
    pc.printf("R is %i \n\r",track_right->get_position());




    output->putData(track_left->get_speed());
    output->putData(track_right->get_speed());
}


void pid(Arguments* input, Reply *output);
//Attach it to an RPC object.
RPCFunction rpc_pid(&pid, "pid");
void pid(Arguments* input, Reply *output)
{
    //one argument: near or far.

    //linescan_query_sensor(linescan_buf);

    //copy linescan_buff into new array, so it doesn't get overwritten.

    float kp = input->getArg<float>();
    float ki = input->getArg<float>();
    float kd = input->getArg<float>();

    track_right->set_gains(kp,ki,kd);
    track_left->set_gains(kp,ki,kd);
}
