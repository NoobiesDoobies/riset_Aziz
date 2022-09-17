/********************************************************
 * Main Program R1
 * Maharani Ayu (KRU 13) & Cristian Napitupulu (KRU 12)
 * Last Modified : 7/5/2022
 ********************************************************/

#include "mbed.h"
#include "Configurations/Variables.h"
#include "Configurations/Declarations.h"
#include "JoystickPS3/JoystickPS3.h"

FileHandle *mbed::mbed_override_console(int fd)
{
    static BufferedSerial serial_port(USBTX, USBRX, 115200);
    return &serial_port;
}

JoystickPS3 stick(UART_TX, UART_RX);

// Arduino
// JoystickPS3 stick(NC, Arduino);

/* For debugging */
float vx_cmd, vy_cmd, w_cmd;
float v_FL_curr, v_FR_curr, v_BR_curr, v_BL_curr;
float FL_pwm, FR_pwm, BR_pwm, BL_pwm;
float FL_target_speed, FR_target_speed, BR_target_speed, BL_target_speed;
float vy_last, vx_last, w_last;
float vy_motor, vx_motor, w_motor;

// int send_spi_inside;
uint8_t state_send;
/*****************/

int main()
{
    us_ticker_init();

    
    stick.setup();
    stick.idle();
    
    // printf("STICK START\n");

    // odom.resetOdom();
    // printf("ODOM START\n");

    // spi_m.frequency(INIT_SCK);
    // spi_m.format(8, 3);
    // cs_m = 1;
    // printf("SPI START\n");
    // printf("Master\n");

    // printf("Hai\n");

    while (1)
    {
        /********************** STICK **********************/

        stick.baca_data();
        
        controlkrai.reverseOtomatis(false);

        if (stick.getMode() == true) {
            // printf("Mode 1\n");
            controlkrai.reverseOtomatis(false);
        }
        else {
            controlkrai.reverseOtomatis(false);
        }

        // FL_motor.speed(0.2);
        // BL_motor.speed(0.2);
        // FR_motor.speed(0.2);
        // BR_motor.speed(0.2);

        if (us_ticker_read() - samplingStick > SAMP_STICK_US){
            // printf("Mode 4\n");
            float vx_cmd, vy_cmd, w_cmd;

            stick.stickState(&vx_cmd, &vy_cmd, &w_cmd);
            controlkrai.set_vx_cmd(vx_cmd);
            controlkrai.set_vy_cmd(vy_cmd);
            controlkrai.set_w_cmd(w_cmd);

            // printf("PS3: vx_cmd: %f, vy_cmd: %f, w_cmd: %f\n", vx_cmd, vy_cmd, w_cmd);

            // stick.sendSPIStickState(&data_send_spi, &data_sampling_spi);

            samplingStick = us_ticker_read();
        }

        /********************** ENCODER **********************/

        if (us_ticker_read() - samplingEncoder > ENC_MOTOR_SAMP_US_DEF)
        {
            // printf("Mode 8\n");
            controlkrai.encoderMotorSamp();
            samplingEncoder = us_ticker_read();
        }

        /********************** PID & ODOM **********************/

        if (us_ticker_read() - samplingPID > PID_MOTOR_SAMP_US)
        {
            // printf("Mode 2\n");
            samplingPID = us_ticker_read();
            controlkrai.pidMotorSamp();
        }

        // if (us_ticker_read() - samplingOdom > SAMP_BASE_SPEED_US)
        // {
        //     // printf("Mode 3\n");
        //     samplingOdom = us_ticker_read();
        //     controlkrai.baseSpeed();
        // }

        if (us_ticker_read() - samplingUpdPos > SAMP_UPD_POS_US)
        {
            // printf("Mode 6\n");
            controlkrai.updatePosition();
            samplingUpdPos = us_ticker_read();
        }

        /********************** MOTOR **********************/

        if (us_ticker_read() - samplingMotor > MOTOR_SAMP_US)
        {
            // printf("Mode 5\n");
            controlkrai.motorSamp();
            samplingMotor = us_ticker_read();
        }

        /********************** IK **********************/

        if (us_ticker_read() - samplingIK > SAMP_IK_US_DEF)
        {
            // printf("Mode 9\n");
            controlkrai.base4Omni();
            samplingIK = us_ticker_read();
        }

        /********************** SEND SPI **********************/

        // if (data_send_spi != 0 && us_ticker_read() - data_sampling_spi > 500)
        // {
        //     // printf("Mode 7\n");
        //     send_spi_inside = data_send_spi;

        //     state_send = unsigned(send_spi_inside);
        //     // printf("%d\n", state_send);
        //     cs_m = 0;
        //     spi_m.write(state_send);
        //     resp = spi_m.write(STATE_IGN);
        //     cs_m = 1;

        //     data_sampling_spi = us_ticker_read();
        //     data_send_spi = 0;

        //     stick.set_send_spi(data_send_spi);
        //     stick.set_sampling_spi(data_sampling_spi);
        // }

        /********************** DEBUG **********************/

        if (us_ticker_read() - samplingPrint > SAMP_PRINT)
        {
            controlkrai.getVars(&vx_cmd, &vy_cmd, &w_cmd, &v_FL_curr, &v_FR_curr, &v_BR_curr, &v_BL_curr, &FL_pwm, &FR_pwm, &BR_pwm, &BL_pwm, &FL_target_speed, &FR_target_speed, &BR_target_speed, &BL_target_speed, &vy_last, &vx_last, &w_last, &vy_motor, &vx_motor, &w_motor);

            // printf("vx_cmd: %f, vy_cmd: %f, w_cmd: %f\n", vx_cmd, vy_cmd, w_cmd);
            // printf("FL_target_speed: %f, FR_target_speed: %f, BR_target_speed: %f, BL_target_speed: %f\n", FL_target_speed, FR_target_speed, BR_target_speed, BL_target_speed);
            // printf("%f, %f, %f, %f, ", FL_target_speed, FR_target_speed, BR_target_speed, BL_target_speed);
            // printf("v_FL_curr: %f, v_FR_curr: %f, v_BR_curr: %f, v_BL_curr: %f\n", v_FL_curr, v_FR_curr, v_BR_curr, v_BL_curr);
            // printf("%f, %f, %f, %f\n", v_FL_curr, v_FR_curr, v_BR_curr, v_BL_curr);
            // printf("FL_pwm: %f, FR_pwm: %f, BR_pwm: %f, BL_pwm: %f\n", FL_pwm, FR_pwm, BR_pwm, BL_pwm);
            printf("enc BL %d BR %d FL %d FR %d\n",encBL.getPulses(),encBR.getPulses(),encFL.getPulses(),encFR.getPulses());
            // printf("X : %d  Y: %d \n", encEx_X.getPulses(), encEx_Y.getPulses());
            // printf("vy_last: %f, vx_last: %f, w_last: %f\n", vy_last, vx_last, w_last);
            // printf("vy_motor: %f, vx_motor: %f, w_motor: %f\n", vy_motor, vx_motor, w_motor);
            printf("\n");
            samplingPrint = us_ticker_read();
        }
    }

    return 1;
}