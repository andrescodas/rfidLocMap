/*
 * TagParticles.h
 *
 *  Created on: 15 janv. 2010
 *      Author: andres
 */

#ifndef TAGPARTICLES_H_
#define TAGPARTICLES_H_

#include "Particles.h"
#include "Tag.h"
#include <map>


class TagParticles: public Particles {
public:
	std::map<Point2D, double,Point2DCmp> particles;
	virtual ~TagParticles();
	void clearWeights();
	void accumulateWeights();
	double normalize();
	void insert(Point2D p, double weight);
	void resample();
	void scale(double factor);
	void sumTagsWeight(TagParticles *tagParticles);
	void copy(TagParticles* tagParticles);
	std::map<Point2D, double,Point2DCmp>::iterator searchParticle(double probability);
	TagParticles();
	TagParticles(Particle _initialPosition);
	int print(const char *fileName);
	void estimatePosition(double *x, double *y, double cov[3]);

};
//a map that allows us to recover a tag (position) by id
typedef std::map<string, TagParticles*> TagParticlesMap;


#endif /* TAGPARTICLES_H_ */
