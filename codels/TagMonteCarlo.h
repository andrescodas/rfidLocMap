/*
 * TagMonteCarlo.h
 *
 *  Created on: 19 janv. 2010
 *      Author: andres
 */

#ifndef TAGMONTECARLO_H_
#define TAGMONTECARLO_H_

//returns the quality, and modifies tagParticles
#include "RFIDSensorModel.h"
#include "RobotParticle.h"
#include "RobotParticles.h"
#include "TagParticles.h"
#include "tagDetection.h"
double weightNormalizeTags(TagDetectionSet* tagDetectionSet,TagParticles* tagParticles,RobotParticles* robotParticles,const char* tagid);


#endif /* TAGMONTECARLO_H_ */
