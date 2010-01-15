#ifndef _TAG_POSITIONS_H_
#define _TAG_POSITIONS_H_

#include "server/rfidLocMapHeader.h"

#include <math.h>
#include "geometricalTools.h"

const double PROBABILITY_OUTSIDE_MODEL = 0;

//ANTENNA POSITIONS PARAMETERS
//  - NUMBER_OF_ANTENNAS -> the number of antennas present on the robot
//  - the only other important structure is the ANTENNA_POSITIONS array that describes the positions of the antenna centers relative to the center of the robot (the angles are relative to the forward direction)
//  - in our case, we use 2 auxiliary structures to describe these positions, since the robot has radial symmetry:
//      - ANTENNA_POSITION_RADIUS - the radius of the circle on which all antenna centers lie
//      - ANTENNA_POSITION_ANGLES - the angles that each center is viewed on from the center of the robot relative to the forward direction

const int NUMBER_OF_ANTENNAS = 8;
double const ANTENNA_POSITION_RADIUS = 0.35;
double const ANTENNA_POSITION_ANGLES[] = { 4* PI / 4, 3* PI / 4, 2* PI / 4, 1*
		PI / 4, 0* PI / 4, 7* PI / 4, 6* PI / 4, 5* PI / 4 };


const char SENSOR_BASE_MODEL[] = "/simulation/sensor_base_model_P.in";

void initSensorModel();

#endif // _TAG_POSITIONS_H_
