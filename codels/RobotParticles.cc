/*
 * RobotParticles.cc
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */
#include <math.h>
#include <stdio.h>
#include "RobotParticles.h"
#include <stdlib.h>

RobotParticles::RobotParticles(int _numberParticles) {
	this->numberParticles = _numberParticles;
}

int RobotParticles::searchParticle(double aleatoryNumber)
{
	int bottomIndex = 0;
	int topIndex = this->numberParticles - 1;
	int currentIndex;

	if (aleatoryNumber < this->particles[0].weight) {
		//printf("\nFirst index Weight == %.30lf\n",p->weights[0]);
		//printf("Warning aleatory number    == %.30lf\n",aleatoryNumber);
		return 0;
	}

	if (aleatoryNumber > this->particles[topIndex].weight) {
		printf("\nWarning top index Weight == %.30lf\n", this->particles[topIndex].weight);
		printf("Warning aleatory number    == %.30lf\n", aleatoryNumber);
		return topIndex;
	}

	while (topIndex - bottomIndex > 1) {
		currentIndex = (int) floor(double((topIndex + bottomIndex) / 2));

		if (this->particles[currentIndex].weight < aleatoryNumber) {
			bottomIndex = currentIndex;
		} else {
			topIndex = currentIndex;
		}
	}

	return topIndex;
}

double RobotParticles::getRandomUniformDouble() {
	return ((double) rand()) / (((double) RAND_MAX));
}

void RobotParticles::resample() {
	int pIndex;
	double aleatoryNumber;
	double weight = double(double(1) / double(this->numberParticles));
	RobotParticles oldParticles(this->numberParticles);

	for (int j = 0; j < this->numberParticles; j++) {
		oldParticles.particles[j] = this->particles[j];
	}

	oldParticles.accumulateWeights();


	for (int k = 0; k < this->numberParticles; k++) {
		aleatoryNumber = getRandomUniformDouble();

		pIndex = oldParticles.searchParticle(aleatoryNumber);

		this->particles[k].x = oldParticles.particles[pIndex].x;
		this->particles[k].y = oldParticles.particles[pIndex].y;
		this->particles[k].theta = oldParticles.particles[pIndex].theta;
		this->particles[k].weight = weight;
	}

}

RobotParticles::RobotParticles(RobotParticle _initialPosition,
		int _numberParticles) {
	double weight = double(double(1) / double(_numberParticles));
	for (int j = 0; j < this->numberParticles; j++) {
		this->particles[j].x = _initialPosition.x;
		this->particles[j].y = _initialPosition.y;
		this->particles[j].theta = _initialPosition.theta;
		this->particles[j].weight = weight;
	}

}

void RobotParticles::normalize() {
	int j;
	double sum = 0;

	for (j = 0; j < this->numberParticles; j++) {
		sum = sum + this->particles[j].weight;
	}

	if (fabs(sum) < 0.00000000000001) {
		printf("Probably producing NaN weights, sum(w) == %lf\n", sum);
	}

	for (j = 0; j < this->numberParticles; j++) {
		this->particles[j].weight = this->particles[j].weight / sum;
	}

}

RobotParticle RobotParticles::estimatePosition() {
	RobotParticle robotParticle;
	double xEstimated = 0;
	double yEstimated = 0;
	double sinEstimated = 0;
	double cosEstimated = 0;

	for (int j = 0; j < this->numberParticles; j++) {
		xEstimated = xEstimated + this->particles[j].x;
		yEstimated = yEstimated + this->particles[j].y;
		sinEstimated = sinEstimated + sin(this->particles[j].theta);
		cosEstimated = cosEstimated + cos(this->particles[j].theta);
	}

	yEstimated = yEstimated / (double) numberParticles;
	cosEstimated = cosEstimated / (double) numberParticles;
	sinEstimated = sinEstimated / (double) numberParticles;

	robotParticle.x = xEstimated / (double) numberParticles;
	robotParticle.y = yEstimated / (double) numberParticles;
	robotParticle.theta = atan2(sinEstimated, cosEstimated);

	this->estimatedPosition = robotParticle;

	return robotParticle;
}

void RobotParticles::accumulateWeights() {
	double sum = 0;

	for (int j = 0; j < this->numberParticles; j++) {
		sum = sum + this->particles[j].weight;
		this->particles[j].weight = sum;
	}

	if (fabs(sum - 1) > 0.00000001) {
		printf("Problems with sum of Weights == %lf\n", sum);
	}

}

void RobotParticles::clearWeights() {
	for (int j = 0; j < this->numberParticles; j++) {
		this->particles[j].weight = 0;
	}
}

RobotParticles::~RobotParticles() {

}
