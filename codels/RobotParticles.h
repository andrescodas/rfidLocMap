/*
 * RobotParticles.h
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#ifndef ROBOTPARTICLES_H_
#define ROBOTPARTICLES_H_

const int MAX_ROBOT_PARTICLES = 1000;

#include "Particles.h"
#include "RobotParticle.h"
#include <vector>


class RobotParticles : public Particles {
private:
	void clearWeights();
	void accumulateWeights();
	RobotParticle estimatePosition();
	void normalize();
	void resample();
	int searchParticle(double probability);
	double getRandomUniformDouble();

public:

	RobotParticle estimatedPosition;
	RobotParticle particles[MAX_ROBOT_PARTICLES];
	int numberParticles;
	RobotParticles(int _numberParticles);
	RobotParticles(RobotParticle _initialPosition,int _numberParticles);
	virtual ~RobotParticles();
};

#endif /* ROBOTPARTICLES_H_ */
