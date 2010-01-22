/*
 * TagMonteCarlo.cc
 *
 *  Created on: 19 janv. 2010
 *      Author: andres
 */
#include "TagMonteCarlo.h"
#include <vector>
#include "RFIDSensorModel.h"
#include "MonteCarloMath.h"
#include "TagExploringParticles.h"
#include "tagDetection.h"

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

double weightTagExploringParticles(TagExploringParticles *exploringParticles,int antenna,bool detected){

	double distance;
	double angle;
	double p;

	for(int particleIt = 0; particleIt < exploringParticles->numberParticles; particleIt++){
		reduceToRobotSensorSystem(0,0,0,exploringParticles->particles[particleIt].x, exploringParticles->particles[particleIt].y,antenna,& distance, & angle);
		p = probabilityModel(distance,angle) ;
		if(!detected){
			p = 1-p;
		}
		exploringParticles->particles[particleIt].weight = exploringParticles->particles[particleIt].weight * p;
	}
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
				printf("Error : quality > 1... in weightNormalizeTags\n");
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
void newTagParticles(TagExploringParticles *exploringParticles,TagDetectionSet *tagDetections,int numberParticles){
	TagDetectionSet::iterator tagDetectionIt;
	int numDetections = 0;
	int antenna;
	int particleIt;
	double distance;
	double angle;
	double newPos[2];
	double weight;
	bool detections[NUMBER_OF_ANTENNAS];

	for(int j = 0; j< NUMBER_OF_ANTENNAS; j++){
		detections[j] = false;
	}

	for(tagDetectionIt = tagDetections->begin(); tagDetectionIt != tagDetections->end();tagDetectionIt++){
		detections[(*tagDetectionIt)->antenna] = true;
		numDetections = numDetections + 1;
	}

	if(numDetections == 0){
		printf("Is not possible to get new particles with no detections\n");
		return;
	}
	weight = double(1.0)/double(numDetections*numberParticles);
	particleIt = 0;
	for(antenna = 0; antenna < NUMBER_OF_ANTENNAS; antenna++){
		if(detections[antenna]){
			for(int i = 0 ; i < numberParticles; i++){
				getProbablePosition(&distance,&angle);

				newPos[0] = cos(angle)*distance+ANTENNA_POSITION_RADIUS;
				newPos[1] = sin(angle)*distance;

				mc_rotation(newPos,ANTENNA_POSITION_ANGLES[antenna]);

				exploringParticles->particles[particleIt].x = newPos[0];
				exploringParticles->particles[particleIt].y = newPos[1];
				exploringParticles->particles[particleIt].weight = weight;
				particleIt++;
			}
		}
	}
	exploringParticles->numberParticles = particleIt;


	for(antenna = 0; antenna < NUMBER_OF_ANTENNAS; antenna ++){
		if(detections[antenna]){
			weightTagExploringParticles(exploringParticles,antenna,true);
		}else{
			weightTagExploringParticles(exploringParticles,antenna,false);
		}
	}
	exploringParticles->normalize();
	exploringParticles->resample(numberParticles);

}

void resampleExploreTags(TagParticles *tagParticles,TagDetectionSet *tagDetections,double inertia,RobotParticles *robotParticles,int numberParticlesTag,bool makeResample){

	bool makeExploration = false;
	bool exploringParticlesInited = false;
	double particlesWeight = double(1.0)/double(numberParticlesTag);

	std::map<Point2D, double,Point2DCmp>::iterator particleIt;
	std::map<Point2D, double,Point2DCmp>::iterator searchResult;

	TagExploringParticles exploringParticles;
	TagDetectionSet::iterator tagDetectionIt;

	double position[2];
	RobotParticle fromParticle;
	Particle particle;

	TagParticles oldParticles;

	if(makeResample){
		tagParticles->copy(&oldParticles);
		tagParticles->particles.clear();
		oldParticles.accumulateWeights();
	}

	if(tagDetections->begin() != tagDetections->end()){
		exploringParticles.numberParticles = numberParticlesTag;
		makeExploration = true;
	}

	if(makeExploration && makeResample){
		for(particleIt = oldParticles.particles.begin();particleIt != oldParticles.particles.end();particleIt++){
			if(mc_getRandomUniformDouble() < inertia){
				searchResult = oldParticles.searchParticle(mc_getRandomUniformDouble());
				tagParticles->insert(searchResult->first,particlesWeight);

			}else{
	            if(!exploringParticlesInited){
	                exploringParticlesInited = true;
	                newTagParticles(&exploringParticles,tagDetections,exploringParticles.numberParticles);
	            }
	            particle = exploringParticles.particles[randInteger(exploringParticles.numberParticles-1)];

	            fromParticle = robotParticles->particles[randInteger(robotParticles->numberParticles-1)];

	            position[0] = particle.x;
	            position[1] = particle.y;

	            mc_rotation(position,fromParticle.theta);

	            tagParticles->insert(Point2D(position[0]+fromParticle.x,position[1]+fromParticle.y),particlesWeight);
			}

		}


	}else if(makeResample){
		for(particleIt = oldParticles.particles.begin();particleIt != oldParticles.particles.end();particleIt++){
				searchResult = oldParticles.searchParticle(mc_getRandomUniformDouble());
				tagParticles->insert(searchResult->first,particlesWeight);
		}


	} else if (makeExploration) {
		exploringParticlesInited = true;
		newTagParticles(&exploringParticles, tagDetections,exploringParticles.numberParticles);

		for(particleIt = oldParticles.particles.begin(); particleIt!= oldParticles.particles.end(); particleIt++) {
            particle = exploringParticles.particles[randInteger(exploringParticles.numberParticles-1)];

            fromParticle = robotParticles->particles[randInteger(robotParticles->numberParticles-1)];

            position[0] = particle.x;
            position[1] = particle.y;

            mc_rotation(position,fromParticle.theta);

            tagParticles->insert(Point2D(position[0]+fromParticle.x,position[1]+fromParticle.y),particlesWeight);
		}
	} else {
		printf("resampleExploreTags -> 'Why are you calling me?' \n");
	}
}

