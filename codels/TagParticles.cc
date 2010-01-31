/*
 * TagParticles.cc
 *
 *  Created on: 15 janv. 2010
 *      Author: andres
 */

#include "TagParticles.h"
#include "math.h"
#include "stdio.h"
#include <stdlib.h>
#include "string.h"


TagParticles::~TagParticles() {

}

TagParticles::TagParticles() {
	this->particles.clear();

}

std::map<Point2D,double,Point2DCmp>::iterator
TagParticles::searchParticle(double probability){

	std::map<Point2D,double,Point2DCmp>::iterator particleIt;

	particleIt = this->particles.begin();

	while(particleIt->second < probability){
		particleIt ++;
	}
	return particleIt;

}

void TagParticles::clearWeights() {

	std::map<Point2D, double>::iterator particlesIt;

	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		particlesIt->second = 0;
		particlesIt++;
	}
}

void TagParticles::accumulateWeights() {

	std::map<Point2D, double>::iterator particlesIt;
	double sum = 0;

	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		sum = sum + particlesIt->second;
		particlesIt->second = sum;
		particlesIt++;
	}

	if (fabs(sum-1) > 0.0000000001 ){
		printf("Problems with sum of particles weight W = %lf\n",sum);
	}
}

double TagParticles::normalize() {

	std::map<Point2D, double>::iterator particlesIt;
	double sum = 0;
	double weight;

	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		sum = sum + particlesIt->second;
		particlesIt++;
	}

	particlesIt = this->particles.begin();

	if(sum < 0.0000000000000001){
		weight = double(1.0)/double(int(this->particles.size()));
		while (particlesIt != this->particles.end()) {
			particlesIt->second = weight;
			particlesIt++;
		}

	}else{

		while (particlesIt != this->particles.end()) {
			particlesIt->second = particlesIt->second / sum;
			particlesIt++;
		}

	}

	return sum;
}

void TagParticles::resample() {
	printf("TagParticles::resample():: Warning: This function is not being implemented \n");

}

void TagParticles::sumTagsWeight(TagParticles *tagParticles){

	std::map<Point2D, double>::iterator particlesIt;
	std::map<Point2D, double>::iterator particlesIt2;

	particlesIt2 = tagParticles->particles.begin();
	for (particlesIt = this->particles.begin(); particlesIt
			!= this->particles.end(); particlesIt++) {

		particlesIt->second = particlesIt->second + particlesIt2->second;

		particlesIt2++;
	}
}

void TagParticles::insert(Point2D p, double weight){

	std::pair<std::map<Point2D, double>::iterator,bool> ret;
	ret = this->particles.insert(std::pair<Point2D,double>(p,weight));

	if(ret.second == false){
		ret.first->second = ret.first->second + weight;
	}
}


void TagParticles::copy(TagParticles* tagParticles) {

	std::map<Point2D, double>::iterator particlesIt;
	tagParticles->particles.clear();

	for (particlesIt = this->particles.begin(); particlesIt
			!= this->particles.end(); particlesIt++) {
		tagParticles->insert(particlesIt->first, particlesIt->second);
	}

}

void TagParticles::scale(double factor) {
	std::map<Point2D, double>::iterator particlesIt;
	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		particlesIt->second = particlesIt->second * factor;
		particlesIt++;
	}
}

TagParticles::TagParticles(Particle _initialPosition){
	this->particles.clear();
	this->insert(Point2D(_initialPosition.x,_initialPosition.y),1);
}

int TagParticles::print(const char *OUTPUT_FILE_NAME) {

	std::map<Point2D, double,Point2DCmp>::iterator particleIt;
	char * pHome;
	char location[128];

	pHome = getenv("HOME");
	strcpy(location, pHome);
	strcat(location, "/simulation/results/");
	strcat(location, OUTPUT_FILE_NAME);

	FILE* outputFile = fopen(location, "w");

	if (outputFile == NULL) {
		printf("Invalid File to write results. File 'path' == %s \n", location);
		return 1;
	}

	fprintf(outputFile, "m = [");

	for (particleIt = this->particles.begin(); particleIt != this->particles.end(); particleIt++) {
		fprintf(outputFile, "\n\t%lf,\t%lf,\t%.20lf;",
				particleIt->first.x, particleIt->first.y,
				particleIt->second);
	}
	fprintf(outputFile, "];");
	fprintf(outputFile, "\nparticules = struct('pos',m);\n");
	fprintf(outputFile, "simulations = [simulations particules];\n");
	fclose(outputFile);
	return 0;
}

void TagParticles::estimatePosition(double *x, double *y, double cov[3]){
	/* Set new variances on (x,y,theta)
		   With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */
		std::map<Point2D, double,Point2DCmp>::iterator particleIt;
		double xEstimated = 0;
		double yEstimated = 0;
		double exy = 0;
		double exx = 0;
		double eyy = 0;

		for (particleIt = this->particles.begin(); particleIt != this->particles.end(); particleIt++) {
			xEstimated = xEstimated + particleIt->first.x * particleIt->second;
			yEstimated = yEstimated + particleIt->first.y * particleIt->second;
			exy += particleIt->first.x * particleIt->first.y * particleIt->second;
			exx += particleIt->first.x * particleIt->first.x * particleIt->second;
			eyy += particleIt->first.y * particleIt->first.y * particleIt->second;
		}

		*x = xEstimated;
		*y = yEstimated;

		/* Set new variances on (x,y,theta)
		   With : v0 = vxx;  v1 = vxy;  v2 = vyy; */

		cov[0] = exx - (*x) * (*x);
		cov[1] = exy - (*x) * (*y);
		cov[2] = eyy - (*y) * (*y);

}


