#ifndef _VARIABLES_H_
#define _VARIABLES_H_

// Comment by: Cris ==========================================================
// Variabel-variabel fisis jangan lupa dipastikan lagi ke mekanik
// sebelum dicoba ke robotnya langsung
// Contoh radius roda, radius base, dll
// ===========================================================================


// ControlKRAI
#ifndef  PI
#define  PI                                     3.14159265358979
#endif

#define  WHEEL_RAD                              0.075
#define  S_TO_US                                1000000
#define  ENC_MOTOR_PULSE                        538
#define  ENC_MOTOR_SAMP_US_DEF                  20000
#define  SAMP_BASE_SPEED_US                     12731
// BASE RECTANGLE
// #define  R_BASE                                 0.312
// BASE SQUARE
#define R_BASE                                  0.3341295

/* INVERSE KINEMATICS */
#define MAX_ACCEL_Y 4
#define MAX_ACCEL_X 7

// Main
#define INIT_SCK 1000000
#define STATE_IGN   0xFF
#define SAMP_UPD_POS_US 12000
#define SAMP_STICK_US 13000
#define MOTOR_SAMP_US 5173
#define SAMP_IK_US_DEF 15000
#define SAMP_PRINT 500000

/* ControlKRAI */
#define ERROR_THRESHOLD 0.1

#endif