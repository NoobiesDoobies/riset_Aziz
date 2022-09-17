#include "ControlKRAI.h"

ControlKRAI::ControlKRAI(Motor *FL_motor, Motor *FR_motor, Motor *BR_motor, Motor *BL_motor, encoderKRAI *encFL, encoderKRAI *encFR, encoderKRAI *encBR, encoderKRAI *encBL, ControlMotor *control_FL_motor, ControlMotor *control_FR_motor, ControlMotor *control_BR_motor, ControlMotor *control_BL_motor, odometriKRAI *odom, pidLo *vxPid, pidLo *vyPid, pidLo *wPid, StanleyPursuit *line, pidLo *pid) {
    this->FL_motor = FL_motor;
    this->FR_motor = FR_motor;
    this->BR_motor = BR_motor;
    this->BL_motor = BL_motor;
    this->encFL = encFL;
    this->encFR = encFR;
    this->encBR = encBR;
    this->encBL = encBL;
    this->control_FL_motor = control_FL_motor;
    this->control_FR_motor = control_FR_motor;
    this->control_BR_motor = control_BR_motor;
    this->control_BL_motor = control_BL_motor;
    this->odom = odom;
    this->vxPid = vxPid;
    this->vyPid = vyPid;
    this->wPid = wPid;
    this->pid = pid;
    this->curr_dest_cout = 1;
    this->mode = 1;
    this->v = 1.0;
    this->line = line;
    this->line->initialPosition(this->odom->position.x, this->odom->position.y, this->odom->position.teta);
    this->line->setError(ERROR_THRESHOLD);
    this->line->setTarget((float)(this->odom->position.x + arr_x_offline_atas_1[1]), (float)(this->odom->position.y + arr_y_offline_atas_1[1]));
    this->otomatis = false;
    this->initialPos.x = this->odom->position.x;
    this->initialPos.y = this->odom->position.y;
}

void ControlKRAI::set_vx_cmd(float vx_cmd) {
    this->vx_cmd = vx_cmd;
}

void ControlKRAI::set_vy_cmd(float vy_cmd) {
    this->vy_cmd = vy_cmd;
}

void ControlKRAI::set_w_cmd(float w_cmd) {
    this->w_cmd = w_cmd;
}

void ControlKRAI::reverseOtomatis(bool otomatis) {
    this->otomatis = otomatis;
}

void ControlKRAI::motorSamp() {
    this->FL_motor->speed(this->FL_pwm);
    this->FR_motor->speed(this->FR_pwm);
    this->BR_motor->speed(this->BR_pwm); 
    this->BL_motor->speed(this->BL_pwm);
}

void ControlKRAI::encoderMotorSamp(){
    this->baseSpeed();
    this->v_FL_curr = (float)this->encFL->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);
    this->v_FR_curr = (float)this->encFR->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);
    this->v_BR_curr = (float)this->encBR->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);
    this->v_BL_curr = (float)this->encBL->getPulses()*2*PI*WHEEL_RAD*S_TO_US/(ENC_MOTOR_PULSE*ENC_MOTOR_SAMP_US_DEF);
    
    /* reset nilai encoder */
    this->encFL->reset();
    this->encFR->reset();
    this->encBR->reset();
    this->encBL->reset();
}

void ControlKRAI::pidMotorSamp(){
    /* menghitung pid motor base */
    this->FL_pwm = this->control_FL_motor->createpwm(this->FL_target_speed, this->v_FL_curr, max_pwm);
    this->FR_pwm = this->control_FR_motor->createpwm(this->FR_target_speed, this->v_FR_curr, max_pwm);
    this->BR_pwm = this->control_BR_motor->createpwm(this->BR_target_speed, this->v_BR_curr, max_pwm);
    this->BL_pwm = this->control_BL_motor->createpwm(this->BL_target_speed, this->v_BL_curr, max_pwm);
}

void ControlKRAI::updatePosition(){
    this->odom->updatePosition();

    // printf("%d\n", this->mode);

    // printf("%f %f %f\t\t%d %d %f %f\n", this->odom->position.x, this->odom->position.y, this->odom->position.teta, mode, curr_dest_cout, arr_x_offline_atas_1[curr_dest_cout], arr_y_offline_atas_1[curr_dest_cout]);

    if (this->otomatis) {
        if (this->line->TargetReached() == true) {
            // printf("HAAAAALLLLLLLLLOOOOOOOOOOOO\n");
            this->line->initialPosition(this->odom->position.x, this->odom->position.y, this->odom->position.teta);
            if (mode == 1) {
                this->line->setTarget((float)(this->initialPos.x + this->arr_x_offline_atas_1[this->curr_dest_cout]), (float)(this->initialPos.y + this->arr_y_offline_atas_1[this->curr_dest_cout]));
            }
            else if (mode == 2) {
                this->line->setTarget((float)(this->initialPos.x + this->arr_x_offline_atas_2[this->curr_dest_cout]), (float)(this->initialPos.y + this->arr_y_offline_atas_2[this->curr_dest_cout]));
            }
            this->curr_dest_cout++;
            if (this->curr_dest_cout == 3 && this->mode == 1) {
                // printf("masuk\n");
                this->mode = 2;
                this->curr_dest_cout = 1;
            }
        }
    }
}

void ControlKRAI::baseSpeed(){
    this->updatePosition();

    // printf("%f %f %f\t\t%d %f %f\n", this->odom->position.x, this->odom->position.y, this->odom->position.teta, curr_dest_cout, arr_x_offline_atas_1[curr_dest_cout], arr_y_offline_atas_1[curr_dest_cout]);

    if (this->otomatis) {
        this->speed_base.x = (this->odom->position.x - this->last_pos.x)*(S_TO_US/SAMP_BASE_SPEED_US);
        this->last_pos.x = this->odom->position.x;
        
        this->speed_base.y = (this->odom->position.y - this->last_pos.y)*(S_TO_US/SAMP_BASE_SPEED_US);
        this->last_pos.y = this->odom->position.y;
        
        this->last_pos.teta = this->odom->position.teta;
    }
}

void ControlKRAI::base4Omni() {
    if (this->otomatis) {
        this->line->updatePosition(this->odom->position.x, this->odom->position.y, this->odom->position.teta, &this->setpoint, &this->feedback, &this->max);
        this->d_out = this->pid->createpwm(0, this->line->getError(), 0.5);
        this->vx_motor = this->v * this->line->getVi() + this->d_out * this->line->getDi();
        this->vy_motor = this->v * this->line->getVj() + this->d_out * this->line->getDj();
        this->w_motor = this->wPid->createpwm(this->setpoint, this->feedback, this->max);
        this->line->setW((-1) * this->w_motor);
    }
    else {
        // Robot jalannya lurus, ga perlu koreksi pake vc vy w PID
        // this->vy_motor = this->vyPid->createpwm(this->vy_cmd, this->speed_base.y, 1);
        // this->vx_motor = this->vxPid->createpwm(this->vx_cmd, this->speed_base.x, 1);
        // this->w_motor = this->wPid->createpwm(this->w_cmd, this->speed_base.teta, 1);
        this->vy_motor = this->vy_cmd;
        this->vx_motor = this->vx_cmd;
        this->w_motor = this->w_cmd;
    }
    
    if(fabs(this->vy_motor - this->vy_last) > 0.015*MAX_ACCEL_Y){
        if (this->vy_last > this->vy_motor){
            this->vy_motor = this->vy_last - MAX_ACCEL_Y*0.015;
        } else {
            this->vy_motor = this->vy_last + MAX_ACCEL_Y*0.015;
        }
    }
    if(fabs(this->vx_motor - this->vx_last) > 0.015*MAX_ACCEL_X){
        if (this->vx_last > this->vx_motor){
            this->vx_motor = this->vx_last - MAX_ACCEL_X*0.015;    
        }
        else{
            this->vx_motor = this->vx_last + MAX_ACCEL_X*0.015;
        }
    }     
    this->vy_last = this->vy_motor;
    this->vx_last = this->vx_motor;

    float vx_motor_input = this->vx_motor*COS30;
    float vy_motor_input = this->vy_motor*COS60;
    float w_motor_input  = this->w_motor*COS15;
    
    this->FL_target_speed = - vy_motor_input - vx_motor_input - w_motor_input * R_BASE;  
    this->FR_target_speed = vy_motor_input - vx_motor_input - w_motor_input * R_BASE;
    this->BR_target_speed = vy_motor_input + vx_motor_input - w_motor_input * R_BASE; 
    this->BL_target_speed = - vy_motor_input + vx_motor_input - w_motor_input * R_BASE; 
}

void ControlKRAI::getVars(float *vx_cmd, float *vy_cmd, float *w_cmd, 
                        float *v_FL_curr, float *v_FR_curr, float *v_BR_curr, float *v_BL_curr, 
                        float *FL_pwm, float *FR_pwm, float *BR_pwm, float *BL_pwm, 
                        float *FL_target_speed, float *FR_target_speed, float *BR_target_speed, float *BL_target_speed, 
                        float *vy_last, float *vx_last, float *w_last, 
                        float *vy_motor, float *vx_motor, float *w_motor) {
    *vx_cmd = this->vx_cmd;
    *vy_cmd = this->vy_cmd;
    *w_cmd = this->w_cmd;
    *v_FL_curr = this->v_FL_curr;
    *v_FR_curr = this->v_FR_curr;
    *v_BR_curr = this->v_BR_curr;
    *v_BL_curr = this->v_BL_curr;
    *FL_pwm = this->FL_pwm;
    *FR_pwm = this->FR_pwm;
    *BR_pwm = this->BR_pwm;
    *BL_pwm = this->BL_pwm;
    *FL_target_speed = this->FL_target_speed;
    *FR_target_speed = this->FR_target_speed;
    *BR_target_speed = this->BR_target_speed;
    *BL_target_speed = this->BL_target_speed;
    *vy_last = this->vy_last;
    *vx_last = this->vx_last;
    *w_last = this->w_last;
    *vy_motor = this->vy_motor;
    *vx_motor = this->vx_motor;
    *w_motor = this->w_motor;
}