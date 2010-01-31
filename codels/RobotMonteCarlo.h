/*
 * RobotMonteCarlo.h
 *
 *  Created on: 17 janv. 2010
 *      Author: andres
 */

#ifndef ROBOTMONTECARLO_H_
#define ROBOTMONTECARLO_H_

#include "tagDetection.h"
#include "Tag.h"
#include "RobotParticles.h"


void movementPrediction(double odo_position[3],double old_odo[3],double odo_cov[3][3],RobotParticles *robotParticles);

void locateRobot(TagDetectionSet* tagDetectionSet,double odo_position[3],double old_odo[3],double odo_cov[3][3],TagMap *tagMap,RobotParticles *robotParticles,double inertia,int step);


#endif /* ROBOTMONTECARLO_H_ */
