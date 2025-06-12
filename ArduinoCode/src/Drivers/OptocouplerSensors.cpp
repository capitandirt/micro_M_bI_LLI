#include "Drivers/OptocouplerSensors.h"

void OptocouplerSensors::init(){
    bitClear(ADCSRA, ADPS0);
    bitClear(ADCSRA, ADPS1);
    bitSet(ADCSRA, ADPS2);

    pinMode(EMITERS_FWD, OUTPUT);
    pinMode(EMITERS_SIDE, OUTPUT);
    pinMode(REC_RIGHT, INPUT);
    pinMode(REC_LEFT, INPUT);
    pinMode(REC_FWD_LEFT, INPUT);
    pinMode(REC_FWD_RIGHT, INPUT);
}

void OptocouplerSensors::tick(){
    // dark read
    static int16_t cur_dark_right = 0;
    cur_dark_right = analogRead(REC_RIGHT);

    if(abs(cur_dark_right - dark_sense.right) < 100){
        dark_sense.right = cur_dark_right;
    }

    dark_sense.left      = analogRead(REC_LEFT);
    dark_sense.forward_l = analogRead(REC_FWD_LEFT);
    dark_sense.forward_r = analogRead(REC_FWD_RIGHT);

    digitalWrite(EMITERS_FWD, 1);
    digitalWrite(EMITERS_SIDE, 1);

    delayMicroseconds(50);

    // light read
    CAN_GET_SENSE = 0;

    // _sense[OptocouplerSense::From::RIGHT] += 
    //     (analogRead(REC_RIGHT) - dark_sense.right - _sense[OptocouplerSense::From::RIGHT]) * K_F;

    // _sense[OptocouplerSense::From::LEFT] +=
    //     (analogRead(REC_LEFT) - dark_sense.left - _sense[OptocouplerSense::From::LEFT]) * K_F;

    // _sense[OptocouplerSense::From::FORWARD_L] +=
    //     (analogRead(REC_FWD_LEFT)  - dark_sense.forward_l - _sense[OptocouplerSense::From::FORWARD_L]) * K_F;

    // _sense[OptocouplerSense::From::FORWARD_R] +=
    //     (analogRead(REC_FWD_RIGHT) - dark_sense.forward_r - _sense[OptocouplerSense::From::FORWARD_R]) * K_F;

    _sense[OptocouplerSense::From::RIGHT] = analogRead(REC_RIGHT) - dark_sense.right;
    _sense[OptocouplerSense::From::LEFT]  = analogRead(REC_LEFT)  - dark_sense.left;

    _sense[OptocouplerSense::From::FORWARD_L] = analogRead(REC_FWD_LEFT)  - dark_sense.forward_l;
        // (analogRead(REC_FWD_LEFT)  - dark_sense.forward_l - _sense[OptocouplerSense::From::FORWARD_L]) * LPF_K;
    _sense[OptocouplerSense::From::FORWARD_R] = analogRead(REC_FWD_RIGHT) - dark_sense.forward_r;
        // (analogRead(REC_FWD_RIGHT) - dark_sense.forward_r - _sense[OptocouplerSense::From::FORWARD_R]) * LPF_K;

    if(_sense[OptocouplerSense::From::FORWARD_L] < 0) _sense[OptocouplerSense::From::FORWARD_L] = 0;
    if(_sense[OptocouplerSense::From::FORWARD_R] < 0) _sense[OptocouplerSense::From::FORWARD_R] = 0;

    CAN_GET_SENSE = 1;

    digitalWrite(EMITERS_FWD, 0);
    digitalWrite(EMITERS_SIDE, 0);
}

void OptocouplerSensors::calc(){
    calc_sense_mask();
    calc_relative_cell();
}

Sense_t OptocouplerSensors::getSense() const{
    while(!CAN_GET_SENSE)
        ;

    return _sense.get();
}

Cell OptocouplerSensors::getRelativeCell() const{
    return _relative_cell;
}
Cell OptocouplerSensors::getCell(Direction robot_dir) const{
    Cell cell_from_sense = inDir(_relative_cell, robot_dir);

    return cell_from_sense;
}

void OptocouplerSensors::printAbsCell() const{
    Serial.print((int)_relative_cell.north_wall);
    Serial.print(" ");
    Serial.print((int)_relative_cell.east_wall);
    Serial.print(" ");
    Serial.print((int)_relative_cell.south_wall);
    Serial.print(" ");
    Serial.println((int)_relative_cell.west_wall);
}

void OptocouplerSensors::printMask() const{
    Serial.println( String(_sense_mask.left) + " " + 
                    String(_sense_mask.forward) + " " +
                    String(_sense_mask.right));
}

void OptocouplerSensors::printSense() const{
    while (!CAN_GET_SENSE)
        ;    

    Serial.println( String(_sense.get().left) + " " + 
                    String(_sense.get().forward_l) + " " +
                    String(_sense.get().forward_r) + " " + 
                    String(_sense.get().right));
}

void OptocouplerSensors::calc_sense_mask(){
    _sense_mask.forward   = max(_sense[OptocouplerSense::From::FORWARD_L], _sense[OptocouplerSense::From::FORWARD_R]) > SENSE_THRESHOLD_FWD_L;
    _sense_mask.left      = _sense[OptocouplerSense::From::LEFT] > SENSE_THRESHOLD_LEFT;
    _sense_mask.right     = _sense[OptocouplerSense::From::RIGHT] > SENSE_THRESHOLD_RIGHT;
}

void OptocouplerSensors::calc_relative_cell(){
    _relative_cell.north_wall = toWallState(_sense_mask.forward);
    _relative_cell.east_wall  = toWallState(_sense_mask.right);
    _relative_cell.west_wall  = toWallState(_sense_mask.left);
    _relative_cell.south_wall = toWallState(false);
}

void OptocouplerSensors::setStaticError()
{
    int16_t right_sense = _sense.get().right;
    int16_t left_sense = _sense.get().left;

    _static_err = right_sense - left_sense;
    _left_sense0 = left_sense;
    _right_sense0 = right_sense;
    SENSE_THRESHOLD_LEFT = left_sense * 0.4;
    SENSE_THRESHOLD_RIGHT = right_sense * 0.4;
}
int16_t OptocouplerSensors::getStaticError() const
{
    return _static_err;
}
int16_t OptocouplerSensors::getRightSense0() const
{
    return _right_sense0;
}
int16_t OptocouplerSensors::getLeftSense0() const
{
    return _left_sense0;
}

int16_t OptocouplerSensors::getLeftTreshold() const
{
    return SENSE_THRESHOLD_LEFT; 
}

int16_t OptocouplerSensors::getRightTreshold() const
{
    return SENSE_THRESHOLD_RIGHT; 
}