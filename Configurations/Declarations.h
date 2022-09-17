#ifndef _DECLARATIONS_H_
#define _DECLARATIONS_H_

#include "ConfigurationPin.h"
#include "../Motor/Motor.h"
#include "../pidLo/pidLo.h"
#include "../SMC_KRAI/SMC_KRAI.h"
#include "../ControlMotor/ControlMotor.h"
#include "../encoderHAL/encoderHAL.h"
#include "../encoderKRAI/encoderKRAI.h"
#include "../odometriKRAI/odometriKRAI.h"
#include "../ControlKRAI/ControlKRAI.h"
#include "../StanleyPursuit/StanleyPursuit.h"
#include "SPI.h"
#include "../encoderHAL/EncoderMspInitF4.h"

/* SAMPLING */
#define PID_MOTOR_SAMP_US 4173

#define MS_TO_US 1000

#define v_batas 1

/* PID MOTOR */     
#define FL_kp  0.9   
#define FL_kp1 1
#define FL_ki  0
#define FL_ki1  0
#define FL_kd  1
#define FL_kd1  0
#define FL_N   0.0
#define FL_TS_ms  7        
#define FL_ka  0.0337 
#define FL_kb  0.0382   

#define FR_kp  0.9
#define FR_kp1 1
#define FR_ki  0
#define FR_ki1  0
#define FR_kd  1
#define FR_kd1  0
#define FR_N   0.0
#define FR_TS_ms  7        
#define FR_ka  0.0337 
#define FR_kb  0.0382 

#define BR_kp  0.9
#define BR_kp1 1
#define BR_ki  0
#define BR_ki1  0
#define BR_kd  1
#define BR_kd1  0
#define BR_N   0.0
#define BR_TS_ms  7        
#define BR_ka  0.0337 
#define BR_kb  0.0382 

#define BL_kp  0.9
#define BL_kp1 1
#define BL_ki  0
#define BL_ki1  0
#define BL_kd  1
#define BL_kd1  0
#define BL_N   0.0
#define BL_TS_ms  7        
#define BL_ka  0.0337 
#define BL_kb  0.038

/* SMC */
#define FL_SMBR_kp 0.002
#define FR_SMBR_kp 0.002
#define BR_SMBR_kp 0.002
#define BL_SMBR_kp 0.002
#define FL_SMBR_kp1 0.001
#define FR_SMBR_kp1 0.001
#define BR_SMBR_kp1 0.001
#define BL_SMBR_kp1 0.001
#define SMC_KSIGMA 1
#define SMC_EPSILON 0.01
#define SMC_BETA 1
#define SMC_SAMPLING 5

std::chrono::microseconds ENC_MOTOR_SAMP_US(20ms);
std::chrono::microseconds SAMP_IK_US(15ms);

/* MOTOR */
// pwm, fwd, rev
Motor FL_motor           (motorFL_PWM, motorFL_R, motorFL_L);
Motor FR_motor           (motorFR_PWM, motorFR_R, motorFR_L);
Motor BR_motor           (motorBR_PWM, motorBR_R, motorBR_L);
Motor BL_motor           (motorBL_PWM, motorBL_R, motorBL_L);

/* PID Vx, Vy, W */
pidLo vxPid(0, 0, 0, SAMP_IK_US_DEF, 10, 1, 1000, 100);
pidLo vyPid(0 , 0, 0, SAMP_IK_US_DEF, 10, 1, 1000, 100);
pidLo wPid(0, 0, 0, SAMP_IK_US_DEF, 10, 1, 1000, 100);

/* PID untuk motor */
pidLo FL_pid_motor(FL_kp, FL_ki, FL_kd, FL_TS_ms, 1, 0, 1000, 100);
pidLo FR_pid_motor(FR_kp, FR_ki, FR_kd, FR_TS_ms, 1, 0, 1000, 100);
pidLo BR_pid_motor(BR_kp, BR_ki, BR_kd, BR_TS_ms, 1, 0, 1000, 100);
pidLo BL_pid_motor(BL_kp, BL_ki, BL_kd, BL_TS_ms, 1, 0, 1000, 100);

/* SMC untuk motor */
SMC FL_smc_motor(FL_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);
SMC FR_smc_motor(FR_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);
SMC BR_smc_motor(BR_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);
SMC BL_smc_motor(BL_SMBR_kp, SMC_KSIGMA, SMC_EPSILON, SMC_BETA, (float)PID_MOTOR_SAMP_US/MS_TO_US, SMC::KECEPATAN);

/*Control Motor */
ControlMotor control_FL_motor(&FL_pid_motor, &FL_smc_motor, v_batas, FL_kp, FL_kp1, FL_kd, FL_kd1, FL_SMBR_kp, FL_SMBR_kp1);
ControlMotor control_FR_motor(&FR_pid_motor, &FR_smc_motor, v_batas, FR_kp, FR_kp1, FR_kd, FR_kd1, FR_SMBR_kp, FR_SMBR_kp1);
ControlMotor control_BR_motor(&BR_pid_motor, &BR_smc_motor, v_batas, BR_kp, BR_kp1, BR_kd, BR_kd1, BR_SMBR_kp, BR_SMBR_kp1);
ControlMotor control_BL_motor(&BL_pid_motor, &BL_smc_motor, v_batas, BL_kp, BL_kp1, BL_kd, BL_kd1, BL_SMBR_kp, BL_SMBR_kp1);

/* ENCODER */
encoderHAL encX(TIM3);
encoderHAL encY(TIM4);

encoderKRAI encFL        (ENC_INTFL_CHA, ENC_INTFL_CHB, 538, X4_ENCODING);
encoderKRAI encFR        (ENC_INTFR_CHA, ENC_INTFR_CHB, 538, X4_ENCODING);
encoderKRAI encBR        (ENC_INTBR_CHA, ENC_INTBR_CHB, 538, X4_ENCODING);
encoderKRAI encBL        (ENC_INTBL_CHA, ENC_INTBL_CHB, 538, X4_ENCODING);

encoderKRAI encEx_X        (ENC_EXT1_CHA, ENC_EXT1_CHB, 538, X4_ENCODING);
encoderKRAI encEx_Y        (ENC_EXT2_CHA, ENC_EXT2_CHB, 538, X4_ENCODING);

/* CMPS */
CMPS12_KRAI cmps12(PB_3, PB_10, 0xC0);

/* ODOMETRI */
odometriKRAI odom(&encX, &encY, &cmps12);

/* TRAJECTORY FOLLOWING */
pidLo pid(0.08, 0.05, 0.5, 0.1 , 0.5, 0, 1000, 100);
StanleyPursuit line;

/* ControlKRAI */
ControlKRAI controlkrai(&FL_motor, &FR_motor, &BR_motor, &BL_motor, &encFL, &encFR, &encBR, &encBL, &control_FL_motor, &control_FR_motor, &control_BR_motor, &control_BL_motor, &odom, &vxPid, &vyPid, &wPid, &line, &pid);

/* TIME SAMPLING */
uint32_t samplingPID = 0;
uint32_t samplingOdom = 0;
uint32_t samplingStick = 0;
uint32_t samplingMotor = 0;
uint32_t samplingUpdPos = 0;
uint32_t samplingEncoder = 0;
uint32_t samplingIK = 0;
uint32_t samplingPrint = 0;

uint32_t data_sampling_spi = 0;
uint8_t data_send_spi = 0;
uint8_t resp = 0;

SPI spi_m(SPI_MOSI, SPI_MISO, SPI_SCK); // mosi, miso, sclk
DigitalOut cs_m(SPI_SS);

/* TICKER */
Ticker encoder_motor_ticker;
Ticker ik_ticker;

#endif