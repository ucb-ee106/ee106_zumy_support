#include "mbed.h"
#include "mbed_rpc.h"
#include "MPU6050.h"
#include "QEI.h"
#include "MODSERIAL.h"
#include "rtos.h"
#include "pindefs.h"
#include "Track.h"

//SerialRPCInterface SerialRPC(USBTX, USBRX, 115200);
MODSERIAL pc(USBTX, USBRX); // tx, rx
//Serial pc(USBTX,USBRX);

float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
int r_enc, l_enc;

RPCVariable<float> rpc_accel_x(&accel_x, "accel_x");
RPCVariable<float> rpc_accel_y(&accel_y, "accel_y");
RPCVariable<float> rpc_accel_z(&accel_z, "accel_z");
RPCVariable<float> rpc_gryo_x(&gyro_x, "gyro_x");
RPCVariable<float> rpc_gryo_y(&gyro_y, "gyro_y");
RPCVariable<float> rpc_gryo_z(&gyro_z, "gyro_z");
RPCVariable<int>   rpc_r_enc(&r_enc, "r_enc");
RPCVariable<int>   rpc_l_enc(&l_enc, "l_enc");
QEI l_wheel (p29, p30, NC, 624);
QEI r_wheel (p11, p12, NC, 624);

MPU6050 mpu6050;

DigitalOut init_done(LED1);
DigitalOut imu_good(LED2);
DigitalOut main_loop(LED3);
DigitalOut test(LED4);

Track track_left(MOTOR_1_1,MOTOR_1_2,p29,p30,624);
Track track_right(MOTOR_2_1,MOTOR_2_2,p11,p12,624);


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
    track_right.invert(true); //I start inverted.

    RtosTimer ledBlinkTimer(led_blink_periodic);
    ledBlinkTimer.start(1000);

    char rpc_input_buf[256];
    char rpc_output_buf[1024];
    //copy-pasta from 192, and remove the parts I don't need.

    pc.baud(115200);



    pc.printf("Hello world! \n\r");
    test = 1;
    wait_ms(100);
    test = 0;
    wait_ms(100);
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
        r_enc=r_wheel.getPulses();
        l_enc=l_wheel.getPulses();
    }

    /*    
    init_done = 0;
    imu_good = 0;
    main_loop = 0;
    test = 0;
    
    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C
    
    volatile bool imu_ready = false;
    
    wait_ms(100);
    
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
    
    if (whoami == 0x68) // WHO_AM_I should always be 0x68
    {
        mpu6050.MPU6050SelfTest(SelfTest);
        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
            mpu6050.initMPU6050();
            mpu6050.getAres();
            mpu6050.getGres();
            imu_ready = true;
            imu_good = 1;
        }
    }
    test = 1;
    init_done = 1;

    uint8_t loop_count = 10;
    while(1) {
        wait_ms(10);
        
        // Handle the encoders
        r_enc=r_wheel.getPulses();
        l_enc=l_wheel.getPulses();
        //pc.printf("Pulses are: %i, %i\r\n", l_enc,r_enc);
        
        if (!(--loop_count)) {
            loop_count = 100;
            main_loop = !main_loop;
        }
        
        if (imu_ready) {
            
            if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
                mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
                mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values

                // Now we'll calculate the accleration value into actual g's
                accel_x = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
                accel_y = (float)accelCount[1]*aRes - accelBias[1];   
                accel_z = (float)accelCount[2]*aRes - accelBias[2];  
               
                // Calculate the gyro value into actual degrees per second
                gyro_x = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
                gyro_y = (float)gyroCount[1]*gRes - gyroBias[1];  
                gyro_z = (float)gyroCount[2]*gRes - gyroBias[2];
            }
        }
    }
    */
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

    track_left.manual_speed(arg0);
    track_right.manual_speed(arg1);

}

void spd(Arguments* input, Reply *output);
//Attach it to an RPC object.
RPCFunction rpc_spd(&spd, "spd");
void spd(Arguments* input, Reply *output)
{
    //one argument: near or far.

    //linescan_query_sensor(linescan_buf);

    //copy linescan_buff into new array, so it doesn't get overwritten.

    pc.printf("L is %i \n\r",track_left.get_position());
    pc.printf("R is %i \n\r",track_right.get_position());




    output->putData(track_left.get_speed());
    output->putData(track_right.get_speed());
}
