#ifndef CONFIGURATIONPIN_H
#define CONFIGURATIONPIN_H

/****************************************/
/********** DEFINE PINLIST HERE *********/
/****************************************/
// TODO RANI : MASUKIN PIN DARI AZIZ
// Motor
#define motorBL_L PA_10
#define motorBL_R PB_13
#define motorBL_PWM PB_14

#define motorFL_R PC_2
#define motorFL_L PC_3
#define motorFL_PWM PB_0

#define motorFR_R PA_14
#define motorFR_L PA_13
#define motorFR_PWM PA_15

#define motorBR_R PB_10
#define motorBR_L PB_15
#define motorBR_PWM PB_1

// Encoder External
#define ENC_EXT1_CHA PB_3 // PC_1//PB_8
#define ENC_EXT1_CHB PB_5 // C_15//PB_9

#define ENC_EXT2_CHA PB_4 // PB_7
#define ENC_EXT2_CHB PA_8 // PB_6

// Encoder Internal
#define ENC_INTFL_CHA PC_0  // PC_12
#define ENC_INTFL_CHB PC_14 // PC_11

#define ENC_INTFR_CHA PC_15
#define ENC_INTFR_CHB PC_1//PC_10

#define ENC_INTBR_CHA PC_10
#define ENC_INTBR_CHB PC_13

#define ENC_INTBL_CHA PC_12 // PC_0
#define ENC_INTBL_CHB PC_11 // PC_14

// I2C
#ifndef I2C_SDA
#define I2C_SDA PB_3
#endif

#ifndef I2C_SCL
#define I2C_SCL PB_10
#endif

// CMPS
#define CMPS_SDA PC_9
#define CMPS_SCL PA_8

// UART
#define UART_RX PA_1
#define UART_TX PA_0

#define Arduino PD_2

// SPI
#ifndef SPI_SCK
#define SPI_SCK PA_5
#endif

#ifndef SPI_SS
#define SPI_SS PA_4
#endif

#ifndef SPI_MISO
#define SPI_MISO PA_6
#endif

#ifndef SPI_MOSI
#define SPI_MOSI PA_7
#endif

#endif