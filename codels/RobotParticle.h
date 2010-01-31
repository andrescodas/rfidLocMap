/*
 * RobotParticle.h
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#ifndef ROBOTPARTICLE_H_
#define ROBOTPARTICLE_H_

#include "Particle.h"

class RobotParticle : public Particle {
public:
	double theta;
	RobotParticle();
	RobotParticle(double _x,double _y,double _theta ,double _weight );
	void print();
	virtual ~RobotParticle();
};

#endif /* ROBOTPARTICLE_H_ */
