/*
 * RobotMonteCarlo.cc
 *
 *  Created on: 17 janv. 2010
 *      Author: andres
 */

#include "RobotMonteCarlo.h"
#include "MonteCarloMath.h"
#include "RFIDSensorModel.h"
#include "geometricalTools.h"


void movementPrediction(double odo_Position[3], double old_odo[3],
		double covariance[3][3], RobotParticles *robotParticles) {

	double deslocInRflexBase[3];
	double distributionPoint[3];
	double auxPoint[3];

	mc_substractVector(odo_Position, old_odo, deslocInRflexBase);

	for (int j = 0; j < robotParticles->numberParticles; j++) {

		mc_generateGaussianRandomPoint(distributionPoint, covariance);

		mc_sumVector(deslocInRflexBase, distributionPoint, auxPoint);

		mc_rotation(auxPoint, robotParticles->particles[j].theta - old_odo[2]);

		robotParticles->particles[j].x = robotParticles->particles[j].x
				+ auxPoint[0];
		robotParticles->particles[j].y = robotParticles->particles[j].y
				+ auxPoint[1];
		robotParticles->particles[j].theta = robotParticles->particles[j].theta
				+ auxPoint[2];

	}

}

double getSimilarityProbability(const TagDetectionSet* tagDetectionSet,const RobotParticle* robotParticle,TagMap* tagMap,double *quality) {

	TagDetection tagDetection;
	TagMap::iterator tagIt;
	TagDetectionSet::iterator tagDetectionIt;


	double p = 1;
	double q = 0;
	double distance;
	double angle;
	double likelihoodAntennaTag;

	tagIt = tagMap->begin();

	while (tagIt != tagMap->end()) {

		strcpy(tagDetection.tagid, tagIt->first);
		for (int a = 0; a < NUMBER_OF_ANTENNAS; a++) {

			reduceToRobotSensorSystem(robotParticle->x, robotParticle->y,
					robotParticle->theta, tagIt->second.x, tagIt->second.y, a,
					&distance, &angle);
			likelihoodAntennaTag = probabilityModel(distance, angle);

			tagDetection.antenna = a;
			tagDetectionIt = tagDetectionSet->find(&tagDetection);

			if (tagDetectionIt != tagDetectionSet->end()) {
				p = p * likelihoodAntennaTag;
			} else {
				p = p * (1 - likelihoodAntennaTag);
			}

			if (p == 0) {
				*quality = 0;
				return 0;
			} else {
				q = q + p;
			}
		}
		tagIt++;
	}
	*quality = q / double(NUMBER_OF_ANTENNAS) / double(tagMap->size());
	return p;
}

double weightRobotParticles(RobotParticles *robotParticles,TagDetectionSet* tagDetectionSet, TagMap* tagMap) {

	double quality = 0;
	double localQuality;
	double p;

	for (int k = 0; k < robotParticles->numberParticles; k++) {

		p = getSimilarityProbability(tagDetectionSet,&(robotParticles->particles[k]),tagMap,&localQuality);
		robotParticles->particles[k].weight = robotParticles->particles[k].weight * p;
		quality = quality + localQuality;

	}

	quality = quality / double(robotParticles->numberParticles);

	return quality;
}


void initRobotPosition(RobotParticle *robotParticle,std::pair<const char* const, Point2D> *tag,int antenna){
	double distance;
	double angleRadians;
	double anglePolar;

	getProbablePosition(&distance,&angleRadians);

    anglePolar = angleWrap((mc_getRandomUniformDouble()-0.5)*2*PI);

    robotParticle->x = tag->second.x + distance*cos(anglePolar);

    robotParticle->y = tag->second.y + distance*sin(anglePolar);

    robotParticle->theta = angleWrap(anglePolar + PI + angleRadians - ANTENNA_POSITION_ANGLES[antenna]);

}



void newRobotParticules(RobotParticles* exploringParticles,TagMap* tagMap,TagDetectionSet* tagDetectionSet){

	int numberDetected = 0;
	int numberParticlesRobot;
	int particleIt;
	TagDetectionSet::iterator tagDetectionIt;
	TagMap::iterator tagIt;

	double weight = double(double(1.0)/double(exploringParticles->numberParticles));

	tagDetectionIt = tagDetectionSet->begin();

	while(tagDetectionIt != tagDetectionSet->end()){
		tagIt = tagMap->find((*tagDetectionIt)->tagid);
		if(tagIt != tagMap->end()){
			numberDetected = numberDetected +  1;
		}
		tagDetectionIt ++;
	}

	if(numberDetected > 0){
		numberParticlesRobot = ceil(double(double(exploringParticles->numberParticles)/double(numberDetected)));
	}else{
		exploringParticles->numberParticles = 0;
		return;
	}

	particleIt = 0;

	tagDetectionIt = tagDetectionSet->begin();
	while (tagDetectionIt != tagDetectionSet->end()) {
		tagIt = tagMap->find((*tagDetectionIt)->tagid);
		if (tagIt != tagMap->end()) {
			for(int i = 0; i < numberParticlesRobot; i++){
				if (particleIt < exploringParticles->numberParticles){
					initRobotPosition(&(exploringParticles->particles[particleIt]),&(*tagIt),(*tagDetectionIt)->antenna);
					exploringParticles->particles[particleIt].weight = weight;
				}else{
					break;
				}
				particleIt ++;
			}
		}
		tagDetectionIt++;
	}


	weightRobotParticles(exploringParticles,tagDetectionSet,tagMap);
	exploringParticles->normalize();
	exploringParticles->resample();
}


void resampleExploreRobot(RobotParticles *robotParticles, double inertia,TagMap* tagMap, TagDetectionSet* tagDetectionSet,bool makeResample) {

	TagDetectionSet::iterator tagDetectionIt;
	TagMap::iterator tagIt;
	RobotParticles oldRobotParticles(robotParticles->numberParticles);
	RobotParticles exploringParticles(robotParticles->numberParticles);

	double particleWeight = double(1.0) / double(robotParticles->numberParticles);

	bool makeExploration = false;
	bool exploringParticlesInited = false;

	if(makeResample){
		robotParticles->copy(&oldRobotParticles);
		oldRobotParticles.accumulateWeights();
	}

	tagDetectionIt = tagDetectionSet->begin();

	while(tagDetectionIt != tagDetectionSet->end()){
		tagIt = tagMap->find((*tagDetectionIt)->tagid);
		if(tagIt != tagMap->end()){
			makeExploration = true;
			break;
		}
		tagDetectionIt ++;
	}

	if (makeExploration && makeResample) {

		for (int k = 0; k < robotParticles->numberParticles; k++) {
			if (mc_getRandomUniformDouble() < inertia) {
				robotParticles->particles[k] = oldRobotParticles.particles[oldRobotParticles.searchParticle(mc_getRandomUniformDouble())];
			} else {
				if (!exploringParticlesInited) {
					exploringParticlesInited = true;
					newRobotParticules(&exploringParticles,tagMap,tagDetectionSet);
				}
				robotParticles->particles[k] = exploringParticles.particles[randInteger(exploringParticles.numberParticles-1)];
			}
		}

	} else if (makeResample) {
		for (int k = 0; k < robotParticles->numberParticles; k++) {
			robotParticles->particles[k]= oldRobotParticles.particles[oldRobotParticles.searchParticle(mc_getRandomUniformDouble())];
		}
	}else{
		newRobotParticules(&exploringParticles,tagMap,tagDetectionSet);
		for (int k = 0; k < robotParticles->numberParticles; k++) {
				robotParticles->particles[k] = exploringParticles.particles[randInteger(exploringParticles.numberParticles-1)];
		}
	}
}


void correctionResampling(RobotParticles *robotParticles, TagMap* tagMap,
		TagDetectionSet* tagDetectionSet, double inertia) {

	double quality;

	quality = weightRobotParticles(robotParticles, tagDetectionSet, tagMap);

	quality = round(quality * 10000) / 10000;

	robotParticles->normalize();

	if (quality == 1) {

	} else if (quality > 1) {
		printf("There is a bug.  Quality > 1");
	} else {

		resampleExploreRobot(robotParticles, inertia + (1 - inertia) * quality,tagMap, tagDetectionSet, true);

		robotParticles->estimatePosition();
	}
}

void locateRobot(TagDetectionSet* tagDetectionSet, double odo_position[3],double old_odo[3], double odo_cov[3][3], TagMap* tagMap,RobotParticles *robotParticles, double inertia) {

	movementPrediction(odo_position, old_odo, odo_cov, robotParticles);
	correctionResampling(robotParticles,tagMap,tagDetectionSet,inertia);

}
