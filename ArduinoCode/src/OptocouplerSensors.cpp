#include "OptocouplerSensors.h"

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
    _sense[OptocouplerSense::From::RIGHT]     = analogRead(REC_RIGHT);
    _sense[OptocouplerSense::From::LEFT]      = analogRead(REC_LEFT);

    _sense[OptocouplerSense::From::FORWARD_L] = analogRead(REC_FWD_LEFT);
    _sense[OptocouplerSense::From::FORWARD_R] = analogRead(REC_FWD_RIGHT);

    digitalWrite(EMITERS_FWD, 1);
    digitalWrite(EMITERS_SIDE, 1);
    delayMicroseconds(50);

    // light read
    _sense[OptocouplerSense::From::RIGHT]     = analogRead(REC_RIGHT) - _sense[OptocouplerSense::From::RIGHT];
    _sense[OptocouplerSense::From::LEFT]      = analogRead(REC_LEFT) - _sense[OptocouplerSense::From::LEFT];

    _sense[OptocouplerSense::From::FORWARD_L] = analogRead(REC_FWD_LEFT) - _sense[OptocouplerSense::From::FORWARD_L];
    _sense[OptocouplerSense::From::FORWARD_R] = analogRead(REC_FWD_RIGHT) - _sense[OptocouplerSense::From::FORWARD_R];

    digitalWrite(EMITERS_FWD, 0);
    digitalWrite(EMITERS_SIDE, 0);
}

Sense_t OptocouplerSensors::getSense(){
    return _sense.get();
}

Cell OptocouplerSensors::getRelativeCell(){
    calc_sense_mask();
    calc_relative_cell();

    return _relative_cell;
}
Cell OptocouplerSensors::getCell(Direction robot_dir){
    calc_sense_mask();
    calc_relative_cell();

    Cell cell_from_sense;
    switch (robot_dir)
    {
    case Direction::N:
        cell_from_sense.north_wall = _relative_cell.north_wall;
        cell_from_sense.east_wall  = _relative_cell.east_wall;
        cell_from_sense.south_wall = _relative_cell.south_wall;
        cell_from_sense.west_wall  = _relative_cell.west_wall;
        break;
    case Direction::E:
        cell_from_sense.north_wall = _relative_cell.west_wall;
        cell_from_sense.east_wall  = _relative_cell.north_wall;
        cell_from_sense.south_wall = _relative_cell.east_wall;
        cell_from_sense.west_wall  = _relative_cell.south_wall;
        break;
    case Direction::S:
        cell_from_sense.north_wall = _relative_cell.south_wall;
        cell_from_sense.east_wall  = _relative_cell.west_wall;
        cell_from_sense.south_wall = _relative_cell.north_wall;
        cell_from_sense.west_wall  = _relative_cell.east_wall;
        break;
    case Direction::W:
        cell_from_sense.north_wall = _relative_cell.east_wall;
        cell_from_sense.east_wall  = _relative_cell.south_wall;
        cell_from_sense.south_wall = _relative_cell.west_wall;
        cell_from_sense.west_wall  = _relative_cell.north_wall;
        break;
    }

    return cell_from_sense;
}

bool OptocouplerSensors::cellIsImpasse(){
    calc_sense_mask();
    calc_relative_cell();

    return toBool(_relative_cell.north_wall) &&
           toBool(_relative_cell.east_wall)  &&
           toBool(_relative_cell.west_wall);
}

void OptocouplerSensors::printAbsCell(){
    calc_sense_mask();
    calc_relative_cell();

    Serial.print((int)_relative_cell.north_wall);
    Serial.print(" ");
    Serial.print((int)_relative_cell.east_wall);
    Serial.print(" ");
    Serial.print((int)_relative_cell.south_wall);
    Serial.print(" ");
    Serial.println((int)_relative_cell.west_wall);
}

void OptocouplerSensors::printMask(){
    calc_sense_mask();

    Serial.println( String(_sense_mask.left) + " " + 
                    String(_sense_mask.forward_l) + " " +
                    String(_sense_mask.forward_r) + " " + 
                    String(_sense_mask.right));
}

void OptocouplerSensors::printSense(){
    Serial.println( String(_sense.get().left) + " " + 
                    String(_sense.get().forward_l) + " " +
                    String(_sense.get().forward_r) + " " + 
                    String(_sense.get().right));
}

void OptocouplerSensors::calc_sense_mask(){
    _sense_mask.forward_l = _sense[OptocouplerSense::From::FORWARD_L] > SENSE_THRESHOLD_FWD_L;
    _sense_mask.forward_r = _sense[OptocouplerSense::From::FORWARD_R] > SENSE_THRESHOLD_FWD_R;
    _sense_mask.left = _sense[OptocouplerSense::From::LEFT] > SENSE_THRESHOLD_LEFT;
    _sense_mask.right = _sense[OptocouplerSense::From::RIGHT] > SENSE_THRESHOLD_RIGHT;
}

void OptocouplerSensors::calc_relative_cell(){
    _relative_cell.north_wall = toWallState(_sense_mask.forward_l || _sense_mask.forward_r);
    _relative_cell.east_wall  = toWallState(_sense_mask.right);
    _relative_cell.west_wall  = toWallState(_sense_mask.left);
    _relative_cell.south_wall = toWallState(false);
}