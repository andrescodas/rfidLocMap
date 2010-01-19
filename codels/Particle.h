/*
 * Particle.h
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

class Particle {
public:
	double x;
	double y;
	double weight;
	Particle();
	Particle(double _x, double _y, double _weight);
	virtual ~Particle();
};

class CompParticle {
public:
	bool operator()(Particle p1, Particle p2) {
		if (p1.x != p2.x) {
			return (p1.x < p2.x);
		} else {
			return (p1.y < p2.y);
		}
	}
};

#endif /* PARTICLE_H_ */
