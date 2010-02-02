/*
 * TagExploringParticles.cpp
 *
 *  Created on: 21 janv. 2010
 *      Author: andres
 */

#include "TagExploringParticles.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

double TagExploringParticles::getRandomUniformDouble() {
	return ((double) rand()) / (((double) RAND_MAX));
}


TagExploringParticles::TagExploringParticles() {

}

TagExploringParticles::~TagExploringParticles() {
}

void TagExploringParticles::accumulateWeights(){

	double sum = 0;

	for (int j = 0; j < this->numberParticles; j++) {
		sum = sum + this->particles[j].weight;
		this->particles[j].weight = sum;
	}

	if (fabs(sum - 1) > 0.00000001) {
		printf("Problems with sum of Weights == %lf\n", sum);
	}

}

double TagExploringParticles::normalize(){
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

	return sum;
}


void TagExploringParticles::clearWeights(){
	for (int j = 0; j < this->numberParticles; j++) {
		this->particles[j].weight = 0.0;
	}
}

void TagExploringParticles::resample()
{
	resample(this->numberParticles);
}


void TagExploringParticles::resample(int _numberNewParticles){

	int pIndex;
	double weight = double(double(1) / double(_numberNewParticles));
	TagExploringParticles oldParticles(this->numberParticles);

	for (int j = 0; j < this->numberParticles; j++) {
		oldParticles.particles[j] = this->particles[j];
	}

	oldParticles.accumulateWeights();


	for (int k = 0; k < _numberNewParticles; k++) {

		pIndex = oldParticles.searchParticle(getRandomUniformDouble());

		this->particles[k].x = oldParticles.particles[pIndex].x;
		this->particles[k].y = oldParticles.particles[pIndex].y;
		this->particles[k].weight = weight;
	}
	this->numberParticles = _numberNewParticles;
}

TagExploringParticles::TagExploringParticles(int _numberParticles){
	this->numberParticles = _numberParticles;
}

int TagExploringParticles::searchParticle(double aleatoryNumber){
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

int TagExploringParticles::print(const char *OUTPUT_FILE_NAME) {


	char * pHome;
	char location[128];

	pHome = getenv("HOME");
	strcpy(location, pHome);
	strcat(location, "/simulation/results/");
	strcat(location, OUTPUT_FILE_NAME);

	FILE* outputFile = fopen(location, "w");

	if (outputFile == NULL) {
		printf("Invalid File to write results. File 'path' == %s \n", location);
		return 1;
	}

	fprintf(outputFile, "m = [");

	for (int i = 0; i < this->numberParticles; i++) {
		fprintf(outputFile, "\n\t%lf,\t%lf,\t%.20lf;", this->particles[i].x,
				this->particles[i].y, this->particles[i].weight);
	}
	fprintf(outputFile, "];");
	fprintf(outputFile, "\nparticules = struct('pos',m);\n");
	fprintf(outputFile, "simulations = [simulations particules];\n");
	fclose(outputFile);
	return 0;

}

