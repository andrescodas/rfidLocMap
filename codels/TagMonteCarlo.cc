/*
 * TagMonteCarlo.cc
 *
 *  Created on: 19 janv. 2010
 *      Author: andres
 */
#include "TagMonteCarlo.h"

double weightTagParticles(TagParticles* tagParticles,RobotParticle *robotParticle,int antenna,bool detected){


	std::map<Point2D, double,Point2DCmp>::iterator particlesIt;
	double distance;
	double quality;
	double angle;
	double p;

	for(particlesIt = tagParticles->particles.begin();particlesIt != tagParticles->particles.end();particlesIt++){

		reduceToRobotSensorSystem(robotParticle->x,robotParticle->y,robotParticle->theta,particlesIt->first.x, particlesIt->first.y,antenna,& distance, & angle);

		p = probabilityModel(distance,angle) ;

		if(!detected){
			p = 1-p;
		}

		particlesIt->second = particlesIt->second * p;
	}

	quality = tagParticles->normalize();
	quality = round(quality*100000)/100000;
	return quality;

}

double weightNormalizeTags(TagDetectionSet* tagDetectionSet,TagParticles* tagParticles,RobotParticles* robotParticles,const char* tagid){

	TagParticles weightedTag;
	TagParticles weightedTagAux;
	TagDetection tagDetection;
	TagDetectionSet::iterator tagDetectionIt;

	strcpy(tagDetection.tagid,tagid);

	tagParticles->copy(&weightedTag);
	weightedTag.clearWeights();

	double localQuality;
	double globalQuality = 0;
	double quality;


	for (int rp = 0; rp < robotParticles->numberParticles; rp++) {
		localQuality = 0;
		tagParticles->copy(&weightedTagAux);

		for (int antenna = 0; antenna < NUMBER_OF_ANTENNAS; antenna++) {
			tagDetection.antenna = antenna;
			tagDetectionIt = tagDetectionSet->find(&tagDetection);

			if (tagDetectionIt != tagDetectionSet->end()) {
				quality = weightTagParticles(&weightedTagAux,
						&robotParticles->particles[rp], antenna, true);
			} else {
				quality = weightTagParticles(&weightedTagAux,
						&robotParticles->particles[rp], antenna, false);
			}

			if(quality > 1){
				printf("Error : quality > 1... in weightNormalizeTags");
			}
			if(quality == 0){
				localQuality = 0;
				break;
			}
			localQuality = localQuality + quality;
		}

		weightedTag.sumTagsWeight(&weightedTagAux);
	    globalQuality = globalQuality + localQuality;
	}
	weightedTag.scale(double(1)/double(robotParticles->numberParticles));

	globalQuality = globalQuality/double(robotParticles->numberParticles)/double(NUMBER_OF_ANTENNAS);
	return globalQuality;

}

