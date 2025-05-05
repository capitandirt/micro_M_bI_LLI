#include "OptocouplerSensors.h"

void OptocouplerSensors::init(){
    pinMode(EMITERS_FWD, OUTPUT);
    pinMode(EMITERS_SIDE, OUTPUT);
    pinMode(REC_RIGHT, INPUT);
    pinMode(REC_LEFT, INPUT);
    pinMode(REC_FWD_LEFT, INPUT);
    pinMode(REC_FWD_RIGHT, INPUT);
}

void OptocouplerSensors::tick(){
    digitalWrite(EMITERS_FWD, 1);
    digitalWrite(EMITERS_SIDE, 1);

    sense[OptocouplerSense::From::RIGHT] = analogRead(REC_RIGHT);
    sense[OptocouplerSense::From::LEFT] = analogRead(REC_LEFT);

    sense[OptocouplerSense::From::FORWARD_L] = analogRead(REC_FWD_LEFT);
    sense[OptocouplerSense::From::FORWARD_R] = analogRead(REC_FWD_RIGHT);
}

Sense_t OptocouplerSensors::getSense(){
    return sense.get();
}

Cell OptocouplerSensors::getCell(Direction robot_dir){
    Cell cell_from_sense;

    calc_sense_mask();

    switch (robot_dir)
    {
    case Direction::N:
        cell_from_sense.north_wall = toWallState(sense_mask.forward_l || sense_mask.forward_r);
        cell_from_sense.east_wall  = toWallState(sense_mask.left);
        cell_from_sense.west_wall  = toWallState(sense_mask.right);
        cell_from_sense.south_wall = toWallState(false);
        break;
    case Direction::S:
        cell_from_sense.north_wall = toWallState(false);
        cell_from_sense.east_wall  = toWallState(sense_mask.right);
        cell_from_sense.west_wall  = toWallState(sense_mask.left);
        cell_from_sense.south_wall = toWallState(sense_mask.forward_l || sense_mask.forward_r);
        break;
    case Direction::E:
        cell_from_sense.north_wall = toWallState(sense_mask.right);
        cell_from_sense.east_wall  = toWallState(sense_mask.forward_l || sense_mask.forward_r);
        cell_from_sense.west_wall  = toWallState(false);
        cell_from_sense.south_wall = toWallState(sense_mask.left);
        break;
    case Direction::W:
        cell_from_sense.north_wall = toWallState(sense_mask.left);
        cell_from_sense.east_wall  = toWallState(false);
        cell_from_sense.west_wall  = toWallState(sense_mask.forward_l || sense_mask.forward_r);
        cell_from_sense.south_wall = toWallState(sense_mask.right);
        break;
    }

    return cell_from_sense;
}

void OptocouplerSensors::printMask(){
    calc_sense_mask();

    Serial.println( String(sense_mask.left) + " " + 
                    String(sense_mask.forward_l) + " " +
                    String(sense_mask.forward_r) + " " + 
                    String(sense_mask.right));
}

void OptocouplerSensors::printSense(){
    Serial.println( String(sense.get().left) + " " + 
                    String(sense.get().forward_l) + " " +
                    String(sense.get().forward_r) + " " + 
                    String(sense.get().right));
}

void OptocouplerSensors::calc_sense_mask(){
    sense_mask.forward_l = sense[OptocouplerSense::From::FORWARD_L] > SENSE_THRESHOLD_FWD_L;
    sense_mask.forward_r = sense[OptocouplerSense::From::FORWARD_R] > SENSE_THRESHOLD_FWD_R;
    sense_mask.left = sense[OptocouplerSense::From::LEFT] > SENSE_THRESHOLD_LEFT;
    sense_mask.right = sense[OptocouplerSense::From::RIGHT] > SENSE_THRESHOLD_RIGHT;
}