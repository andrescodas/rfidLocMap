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

const int MAX_TAG_PARTICLES = 500;


class TagParticles: public Particles {
public:
	std::map<Point2D, double,Point2DCmp> particles;
	virtual ~TagParticles();
	int numberParticles;
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
	TagParticles(int _numberParticles);
	TagParticles(int _numberParticles, Particle _initialPosition);



};

//a map that allows us to recover a tag (position) by id
typedef std::map<const char*, TagParticles*, strCmp> TagParticlesMap;


#endif /* TAGPARTICLES_H_ */
