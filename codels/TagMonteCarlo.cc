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

		//TODO: ANTENNA 5 OUT OF ORDER
		if(antenna == 5){
			p = 0.01;
		}

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

		//TODO: ANTENNA 5 OUT OF ORDER
		if(antenna == 5){
			p = 0.01;
		}

		if(!detected){
			p = 1-p;
		}
		exploringParticles->particles[particleIt].weight = exploringParticles->particles[particleIt].weight * p;
	}
}


double weightNormalizeTags(TagDetectionSet* tagDetectionSet,TagParticles* tagParticles,RobotParticles* robotParticles,string tagid){

	TagParticles weightedTag;
	TagParticles weightedTagAux;
	TagDetection tagDetection;
	TagDetectionSet::iterator tagDetectionIt;

	tagDetection.tagid = tagid;

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
	int k;
	double inertiaRatio;
	double oldparticles = 0;
	double newparticles = 0;
	std::map<Point2D, double,Point2DCmp>::iterator searchResult;

	TagExploringParticles exploringParticles;


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
		for(k = 0;k < numberParticlesTag;k++){
			if(mc_getRandomUniformDouble() < inertia){
				searchResult = oldParticles.searchParticle(mc_getRandomUniformDouble());
				tagParticles->insert(searchResult->first,particlesWeight);
				newparticles = newparticles + 1;
			}else{
	            if(!exploringParticlesInited){
	                exploringParticlesInited = true;
	                newTagParticles(&exploringParticles,tagDetections,exploringParticles.numberParticles);
	            }
	            particle = exploringParticles.particles[randInteger(exploringParticles.numberParticles)];

	            fromParticle = robotParticles->particles[randInteger(robotParticles->numberParticles)];

	            position[0] = particle.x;
	            position[1] = particle.y;

	            mc_rotation(position,fromParticle.theta);

	            tagParticles->insert(Point2D(position[0]+fromParticle.x,position[1]+fromParticle.y),particlesWeight);
	            oldparticles = oldparticles + 1;
			}

		}
			inertiaRatio = newparticles/(newparticles+oldparticles);
			printf("ratio comparison %lf == %lf\n",inertia,inertiaRatio);

	}else if(makeResample){
		for(k = 0;k < numberParticlesTag;k++){
				searchResult = oldParticles.searchParticle(mc_getRandomUniformDouble());
				tagParticles->insert(searchResult->first,particlesWeight);
		}


	} else if (makeExploration) {
		exploringParticlesInited = true;
		newTagParticles(&exploringParticles, tagDetections,exploringParticles.numberParticles);

		for(k = 0;k < numberParticlesTag;k++) {
            particle = exploringParticles.particles[randInteger(exploringParticles.numberParticles)];

            fromParticle = robotParticles->particles[randInteger(robotParticles->numberParticles)];

            position[0] = particle.x;
            position[1] = particle.y;

            mc_rotation(position,fromParticle.theta);

            tagParticles->insert(Point2D(position[0]+fromParticle.x,position[1]+fromParticle.y),particlesWeight);
		}
	} else {
		printf("resampleExploreTags -> 'Why are you calling me?' \n");
	}
}

void locateTags(TagParticlesMap *inferringTags,RobotParticles *robotParticles,TagDetectionSet *inferringTagsDetectionSet,double inertiaTag,int numberParticlesTag,int step) {

	TagDetectionSet knownInferringTagsDetection;
	TagDetectionSet newInferringTagsDetection;
	TagDetectionSet::iterator detectionIt;
	TagDetectionSet tagDetectionIt;
	TagParticles *newDetectedTag;
	std::pair<string,TagParticles*> newTag;
	double quality;
	char outputFile[64];


	sortDetectionsByTagParticlesMap(&knownInferringTagsDetection,
			&newInferringTagsDetection, inferringTagsDetectionSet,
			inferringTags);

	for (TagParticlesMap::iterator iTagIt = inferringTags->begin(); iTagIt
			!= inferringTags->end(); iTagIt++) {

		tagDetectionIt.clear();
		sortDetectionsByTagid(&tagDetectionIt, &knownInferringTagsDetection,
				iTagIt->first);

				sprintf(outputFile, "TaginitStep%d.m",step);
				iTagIt->second->print(outputFile);

		quality = weightNormalizeTags(&tagDetectionIt, (iTagIt->second),
				robotParticles, iTagIt->first);

				sprintf(outputFile, "Tagweighted%d.m",step);
				iTagIt->second->print(outputFile);


		if (quality > 1) {
			printf("WeightNormalizeTags with quality > 1\n");
		} else {
			printf("  Tag Particles Quality == %lf\n", quality);
		}

		resampleExploreTags((iTagIt->second), &tagDetectionIt, inertiaTag + (1
				- inertiaTag) * quality, robotParticles, numberParticlesTag,
				true);

				sprintf(outputFile, "Tagresampled%d.m",step);
				iTagIt->second->print(outputFile);


	}
	printf("WeightTags\n");
	detectionIt = newInferringTagsDetection.begin();
	while (detectionIt != newInferringTagsDetection.end()) {

		tagDetectionIt.clear();
		sortDetectionsByTagid(&tagDetectionIt, &newInferringTagsDetection,
				(*detectionIt)->tagid);

		newDetectedTag = new TagParticles();

		resampleExploreTags(newDetectedTag, &tagDetectionIt, 0,
				robotParticles, numberParticlesTag, false);

		newDetectedTag->print("Tagnewparticles.m");

		newTag.first = (*detectionIt)->tagid;
		newTag.second = newDetectedTag;
		inferringTags->insert(newTag);

		for (detectionIt = tagDetectionIt.begin(); detectionIt
				!= tagDetectionIt.end(); detectionIt++) {
			newInferringTagsDetection.erase((*detectionIt));
		}
		detectionIt = newInferringTagsDetection.begin();
	}

}
