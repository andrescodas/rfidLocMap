/*
 * TagParticles.cc
 *
 *  Created on: 15 janv. 2010
 *      Author: andres
 */

#include "TagParticles.h"

TagParticles::TagParticles() {

}

void TagParticles::clearWeights() {

	set<Particle*, CompParticle>::iterator particlesIt;

	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		(*particlesIt)->weight = 0;
	}
}

void TagParticles::accumulateWeights() {

	set<Particle*, CompParticle>::iterator particlesIt;
	double sum = 0;

	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		sum = sum + (*particlesIt)->weight;
		(*particlesIt)->weight = sum;
	}

	if (fabs(sum-1) > 0.0000000001 ){
		printf("Problems with sum of particles weight W = %lf",sum);
	}


}

void TagParticles::normalize() {

	set<Particle*, CompParticle>::iterator particlesIt;
	double sum = 0;

	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		sum = sum + (*particlesIt)->weight;
	}

	particlesIt = this->particles.begin();
	while (particlesIt != this->particles.end()) {
		(*particlesIt)->weight = (*particlesIt)->weight/sum;
	}

}

void TagParticles::resample() {


}

TagParticles::~TagParticles() {
// TODO Auto-generated destructor stub
}
