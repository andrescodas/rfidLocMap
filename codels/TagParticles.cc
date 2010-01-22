/*
 * TagParticles.cc
 *
 *  Created on: 15 janv. 2010
 *      Author: andres
 */

#include "TagParticles.h"
#include "math.h"
#include "stdio.h"



TagParticles::~TagParticles() {

}

TagParticles::TagParticles() {
}

std::map<Point2D,double,Point2DCmp>::iterator TagParticles::searchParticle(double probability){

	std::map<Point2D,double,Point2DCmp>::iterator particleIt;

	particleIt = this->particles.begin();

	while(particleIt->second < probability){
		particleIt ++;
	}
	return particleIt;

}

TagParticles::TagParticles(int _numberParticles) {
	this->numberParticles = _numberParticles;
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
		printf("Problems with sum of particles weight W = %lf",sum);
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

	tagParticles->numberParticles = this->numberParticles;
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
