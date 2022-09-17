#ifndef _CONTROL_KRAI_H_
#define _CONTROL_KRAI_H_

// FOR RECTANGLE BASE
// #define COS45 0.70710678118
// #define COS14_53 -0.382807

#define max_pwm 1.0

// FOR SQUARE BASE
#define COS15 0.96592582628
#define COS30 0.86602540378
#define COS60 0.5

#include "../Configurations/Variables.h"
#include "../odometriKRAI/Coordinate.h"
#include "../odometriKRAI/odometriKRAI.h"
#include "../Motor/Motor.h"
#include "../ControlMotor/ControlMotor.h"
#include "../encoderHAL/encoderHAL.h"
#include "../encoderKRAI/encoderKRAI.h"
#include "../StanleyPursuit/StanleyPursuit.h"
#include "../pidLo/pidLo.h"

class ControlKRAI {
    private:
        Coordinate last_pos;
        Coordinate speed_base;

        /* MOTOR */
        // pwm, fwd, rev
        Motor *FL_motor;
        Motor *FR_motor;
        Motor *BR_motor;
        Motor *BL_motor;

        /*Control Motor */
        ControlMotor *control_FL_motor;
        ControlMotor *control_FR_motor;
        ControlMotor *control_BR_motor;
        ControlMotor *control_BL_motor;

        /* ENCODER */
        encoderKRAI *encFL;
        encoderKRAI *encFR;
        encoderKRAI *encBR;
        encoderKRAI *encBL;

        /* ODOMETRI */
        odometriKRAI *odom;

        pidLo *vxPid;
        pidLo *vyPid;
        pidLo *wPid;

        /* TRAJECTORY FOLLOWER */
        /* Rulebook online -> 1 gerak 3800 mm, 2 gerak 5300 mm */
        /* 1 -> gerak di sisi panjang lapangan */
        /* 2 -> gerak di sisi lebar lapangan */
        float arr_x_online[3] = {0, 3.8, 3.8};
        float arr_y_online[3] = {0, 0, 5.3};
        
        // Ke arah ball rack di bagian atas di figure rulebook
        // 1 itu gerak ke arah ball rack
        // 2 itu gerak kembali ke SZ
        float arr_x_offline_atas_1[3] = {0, 0, 5.5};
        float arr_y_offline_atas_1[3] = {0, -5.25, -5.25};
        float arr_x_offline_atas_2[3] = {5.5, 0, 0};
        float arr_y_offline_atas_2[3] = {-5.25, -5.25, 0};

        // Ke arah ball rack di bagian bawah di figure rulebook
        // 1 itu gerak ke arah ball rack
        // 2 itu gerak kembali ke SZ
        float arr_x_offline_bawah_1[3] = {0, 0, 5.5};
        float arr_y_offline_bawah_1[3] = {0, 5.25, 5.25};
        float arr_x_offline_bawah_2[3] = {5.5, 0, 0};
        float arr_y_offline_bawah_2[3] = {5.25, 5.25, 0};

        Coordinate initialPos;

        StanleyPursuit *line;
        pidLo *pid;
        int curr_dest_cout;
        int mode; // mode 1 pakai ke ball rack, mode 2 pakai ke SZ
        // Perlu 1 variabel lagi buat atas atau bawah

        float setpoint;
        float feedback;
        float max;
        float d_out;
        float v;

        /* Manual/Automatic settings */
        bool otomatis;

        /* INTERNAL VARIABLES */
        float vx_cmd, vy_cmd, w_cmd;
        float v_FL_curr, v_FR_curr, v_BR_curr, v_BL_curr;
        float FL_pwm, FR_pwm, BR_pwm, BL_pwm;
        float FL_target_speed, FR_target_speed, BR_target_speed, BL_target_speed;
        float vy_last, vx_last, w_last;
        float vy_motor, vx_motor, w_motor;

    public:
        ControlKRAI(Motor *FL_motor, Motor *FR_motor, Motor *BR_motor, Motor *BL_motor, encoderKRAI *encFL, encoderKRAI *encFR, encoderKRAI *encBR, encoderKRAI *encBL, ControlMotor *control_FL_motor, ControlMotor *control_FR_motor, ControlMotor *control_BR_motor, ControlMotor *control_BL_motor, odometriKRAI *odom, pidLo *vxPid, pidLo *vyPid, pidLo *wPid, StanleyPursuit *line, pidLo *pid);
        void set_vx_cmd(float vx_cmd);
        void set_vy_cmd(float vy_cmd);
        void set_w_cmd(float w_cmd);
        void reverseOtomatis(bool otomatis);
        /* 
        * prosedur untuk melakukan sampling enc motor
        * */
        void motorSamp();
        void encoderMotorSamp();
        void pidMotorSamp();
        void updatePosition();
        void baseSpeed();
        void base4Omni();

        void getVars(float *vx_cmd, float *vy_cmd, float *w_cmd, float *v_FL_curr, float *v_FR_curr, float *v_BR_curr, float *v_BL_curr, float *FL_pwm, float *FR_pwm, float *BR_pwm, float *BL_pwm, float *FL_target_speed, float *FR_target_speed, float *BR_target_speed, float *BL_target_speed, float *vy_last, float *vx_last, float *w_last, float *vy_motor, float *vx_motor, float *w_motor);
};

#endif