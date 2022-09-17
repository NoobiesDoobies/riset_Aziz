/***************************************************************************
 * Title      : Library untuk odometri base
 * Name       : odometriKrai.cpp
 * Version    : 1.0
 * Author     : Oktavianus Irvan Sitanggang EL 18
 * Date       : 17 Desember 2019
 * Description:
 *
 * Library ini digunakan untuk melakukan perhitungan posisi robot
 * 
 * 
 ***************************************************************************/

/******************************* library ***********************************/

#include "odometriKRAI.h"

// Constants
#ifndef PI
#define PI  3.14159265359
#endif

#ifndef D_RODA
#define D_RODA 0.06
#endif
/*************************** inisiasi class *******************************/
odometriKRAI::odometriKRAI(encoderHAL *encX, encoderHAL *encY, CMPS12_KRAI *kompass) {
    this->position.x = 0;      // initiate all Value
    this->position.y = 0;      
    this->position.teta = 0;
    this->encX = encX;
    this->encY = encY;
    this->kompass = kompass;
    this->kompass->compassResetOffsetValue();
}

/*************************** definisi fungsi ******************************/
/* update position from base */
void odometriKRAI::updatePosition(void){
    int xTemp = this->encX->getPulses(1);                            /* butuh 1.5us */
    int yTemp = this->encY->getPulses(1);                            /* butuh 1.5us */

    this->kompass->compassUpdateValue();                               /* ??? */
    // printf("CompassValue : %f\n", this->kompass->compassValue());
    this->position.teta = this->kompass->compassValue();
    this->position.x += -((float)xTemp*PI*D_RODA/2400)*cos(this->position.teta) - ((float)yTemp*PI*D_RODA/2400)*-sin(this->position.teta);        /* butuh 4.5 us */
    this->position.y += -((float)xTemp*PI*D_RODA/2400)*sin(this->position.teta) - ((float)yTemp*PI*D_RODA/2400)*cos(this->position.teta);         /* butuh 4.5 us */
    // printf("Pos: %f %f %f\n", this->position.x, this->position.y, this->position.teta);
}

void odometriKRAI::setPosition(float x_input, float y_input, float theta_input){
    this->position.x = x_input;
    this->position.y = y_input;
    
    this->kompass->changeThetaValue(theta_input);
    this->position.teta = kompass->compassValue();
    // printf("CompassValue set position : %f\n", this->kompass->compassValue());
}

/* to reset all the position */
void odometriKRAI::resetOdom(void){
    this->position.x = 0;      // initiate all Value
    this->position.y = 0;      
    this->position.teta = 0;
    
    this->kompass->compassResetOffsetValue();
}



