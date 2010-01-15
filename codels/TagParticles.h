/*
 * TagParticles.h
 *
 *  Created on: 15 janv. 2010
 *      Author: andres
 */

#ifndef TAGPARTICLES_H_
#define TAGPARTICLES_H_

#include "Particles.h"
#include <set>

const int MAX_TAG_PARTICLES = 500;

class TagParticles: public Particles {
private:
	int numberParticles;
	set <Particle*, CompParticle> particles;
	void clearWeights();
	void accumulateWeights();
	void normalize();
	void resample();
public:
	TagParticles(int numberParticles);
	TagParticles(int _numberParticles, Particle _initialPosition);
	virtual ~TagParticles();


};

#endif /* TAGPARTICLES_H_ */
