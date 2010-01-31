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
#include "string.h"
#include "geometricalTools.h"

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


	double weight = double(double(1) / double(this->numberParticles));
	RobotParticles oldParticles(this->numberParticles);

	for (int j = 0; j < this->numberParticles; j++) {
		oldParticles.particles[j] = this->particles[j];
	}

	oldParticles.accumulateWeights();

	for (int k = 0; k < this->numberParticles; k++) {
		this->particles[k] = oldParticles.particles[oldParticles.searchParticle(getRandomUniformDouble())];
		this->particles[k].weight = weight;
	}

}

RobotParticles::RobotParticles(RobotParticle _initialPosition,int _numberParticles) {
	double weight = double(double(1) / double(_numberParticles));
	this->numberParticles = _numberParticles;
	for (int j = 0; j < this->numberParticles; j++) {
		this->particles[j].x = _initialPosition.x;
		this->particles[j].y = _initialPosition.y;
		this->particles[j].theta = _initialPosition.theta;
		this->particles[j].weight = weight;
	}

}

double RobotParticles::normalize() {
	int j;
	double sum = 0;
	double weight;

	for (j = 0; j < this->numberParticles; j++) {
		sum = sum + this->particles[j].weight;
	}

	if (fabs(sum) < 0.00000000000001) {
		printf("Robot Particles: sum(w) == %lf, reseting weights\n", sum);
		weight = double(double(1) / double(this->numberParticles));
		for (j = 0; j < this->numberParticles; j++) {
			this->particles[j].weight = weight;
		}
	} else {
		for (j = 0; j < this->numberParticles; j++) {
			this->particles[j].weight = this->particles[j].weight / sum;
		}
	}
	return sum;
}

void RobotParticles::estimatePosition(double *x, double *y, double *theta,double cov[6]) {
	/* Set new variances on (x,y,theta)
	   With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */

	double xEstimated = 0;
	double yEstimated = 0;
	double sinEstimated = 0;
	double cosEstimated = 0;
	double exy = 0;
	double exx = 0;
	double eyy = 0;
	double ext = 0;
	double eyt = 0;
	double ett = 0;
	double desvTheta_aux;

	for (int j = 0; j < this->numberParticles; j++) {
		xEstimated = xEstimated + this->particles[j].x;
		yEstimated = yEstimated + this->particles[j].y;
		sinEstimated = sinEstimated + sin(this->particles[j].theta);
		cosEstimated = cosEstimated + cos(this->particles[j].theta);
		exy += this->particles[j].x * this->particles[j].y;
		exx += this->particles[j].x * this->particles[j].x;
		eyy += this->particles[j].y * this->particles[j].y;
	}


	cosEstimated = cosEstimated / (double) numberParticles;
	sinEstimated = sinEstimated / (double) numberParticles;

	*x = xEstimated / (double) numberParticles;
	*y = yEstimated / (double) numberParticles;
	*theta = atan2(sinEstimated, cosEstimated);

	exy = exy / (double) numberParticles;
	exx = exx / (double) numberParticles;
	eyy = eyy / (double) numberParticles;

	/* Set new variances on (x,y,theta)
	   With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */

	cov[0] = exx - (*x) * (*x);
	cov[1] = exy - (*x) * (*y);
	cov[2] = eyy - (*y) * (*y);

	for (int i = 0; i < numberParticles; ++i) {
		desvTheta_aux = angleWrap(this->particles[i].theta - *theta);

		ext += this->particles[i].x * desvTheta_aux;
		eyt += this->particles[i].y * desvTheta_aux;
		ett += desvTheta_aux * desvTheta_aux;

	}
	ext = ext / (double) numberParticles;
	eyt = eyt / (double) numberParticles;
	ett = ett / (double) numberParticles;


	cov[3] = ext - (*x) * (*theta);
	cov[4] = eyt - (*y) * (*theta);
	cov[5] = ett;

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

void RobotParticles::copy(RobotParticles* newRobotParticles){

	newRobotParticles->numberParticles = this->numberParticles;

	for(int j = 0; j < this->numberParticles; j++){
		newRobotParticles->particles[j] = this->particles[j];
	}

}

RobotParticles::~RobotParticles(){
}

int RobotParticles::print(const char *OUTPUT_FILE_NAME){

	char * pHome;
	char location[128];

	pHome = getenv("HOME");
	strcpy(location, pHome);
	strcat(location, "/simulation/results/");
	strcat(location, OUTPUT_FILE_NAME);

	FILE* outputFile = fopen(location, "w");

	if (outputFile == NULL) {
		printf("Invalid File to write results. File 'path' == %s \n", location);
	}

	fprintf(outputFile, "m = [");

	for (int i = 0; i < this->numberParticles; i++) {

		fprintf(outputFile, "\n\t%.10lf,\t%.10lf,\t%.10lf,\t%.20lf;", this->particles[i].x,
				this->particles[i].y,this->particles[i].theta,this->particles[i].weight);
	}
	fprintf(outputFile, "];");
	fprintf(outputFile, "\nparticules = struct('pos',m);\n");
	fprintf(outputFile, "simulations = [simulations particules];\n");
	fclose(outputFile);

	return 0;
}

void RobotParticles::show()
{

	for (int i = 0; i < this->numberParticles; i++) {
		printf("\n\t%.10lf,\t%.10lf,\t%.10lf,\t%.20lf;", this->particles[i].x,this->particles[i].y,this->particles[i].theta,this->particles[i].weight);
	}

}




