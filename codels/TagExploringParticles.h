/*
 * TagExploringParticles.h
 *
 *  Created on: 21 janv. 2010
 *      Author: andres
 */

#ifndef TAGEXPLORINGPARTICLES_H_
#define TAGEXPLORINGPARTICLES_H_

#include "Particles.h"

const int MAX_EXPLORING_PARTICLES = 8000;

class TagExploringParticles: public Particles {
private:
	double getRandomUniformDouble();
public:
	int numberParticles;
	Particle particles[MAX_EXPLORING_PARTICLES];
	TagExploringParticles();
	TagExploringParticles(int _numberParticles);
	virtual ~TagExploringParticles();
	void accumulateWeights();
	double normalize();
	void resample(int _numberNewParticles);
	int searchParticle(double probability);
	void clearWeights();
	void resample();
	int print(const char *filename);
};

#endif /* TAGEXPLORINGPARTICLES_H_ */
