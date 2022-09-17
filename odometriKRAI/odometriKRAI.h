#ifndef ODOMETRIKRAI_H
#define ODOMETRIKRAI_H

#include "mbed.h"
#include "../encoderHAL/encoderHAL.h"
#include "../CMPS12_KRAI/CMPS12_KRAI.h"
#include "Coordinate.h"

class odometriKRAI{
    private:
        encoderHAL *encX;
        encoderHAL *encY;
        CMPS12_KRAI *kompass;
    public:
        odometriKRAI(encoderHAL *encX, encoderHAL *encY, CMPS12_KRAI *kompass);
        Coordinate position;
        void resetOdom(void);
        void updatePosition(void);
        void setPosition(float x_input, float y_input, float theta_input);
};


#endif