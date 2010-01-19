/*
 * Particles.h
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#ifndef PARTICLES_H_
#define PARTICLES_H_

#include "Particle.h"

class Particles {
public:
	virtual void clearWeights() = 0;
	virtual void accumulateWeights() = 0;
	virtual double normalize() = 0;
	virtual void resample() = 0;
};

#endif /* PARTICLES_H_ */
