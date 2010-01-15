
#include "RFIDSensorModel.h"
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <string>
#include "geometricalTools.h"



int modelSize[2];
double* sensorModel = NULL; // this is really a matrix but represent in a lineal vector
// discretization(in meters) of the matrix representation
double stepDistance;
double stepRadius;

//generates the model for all the antennas
void initSensorModel() {

	FILE* inputFile;
	int scanReturn;
	char * pHome;
	char location[128];

	pHome = getenv("HOME");
	strcpy(location, pHome);
	strcat(location, SENSOR_BASE_MODEL);
	inputFile = fopen(location, "r");

	if (inputFile == NULL) {
		fprintf(
				stderr,
				"[generateSensorModel] ERROR: Could not open the file containing the base model (%s)\n",
				SENSOR_BASE_MODEL);
	} else {

		scanReturn = fscanf(inputFile, "%u", &modelSize[0]);
		scanReturn = fscanf(inputFile, "%u", &modelSize[1]);
		scanReturn = fscanf(inputFile, "%lf", &stepDistance);
		scanReturn = fscanf(inputFile, "%lf", &stepRadius);

		sensorModel = (double*) malloc(modelSize[0] * modelSize[1]
				* sizeof(double));

		for (int i = 0; i < modelSize[0]; i++) {
			for (int j = 0; j < modelSize[1]; j++) {
				scanReturn = fscanf(inputFile, "%lf", &sensorModel[modelSize[1]
						* i + j]);
			}
		}
	}

}

double probabilityModel(double distance, double angle) {

	int d = (int) floor(distance / stepDistance);
	int a = (int) floor(fabs(angle) / stepRadius);

	// in case angle a == PI it will be always outside the model!
	if (fabs(angle) == PI) {
		a = a - 1;
	}

	if (((d + 1) > modelSize[0]) || ((a + 1) > modelSize[1])) { // if outsideModel]
		return PROBABILITY_OUTSIDE_MODEL;
	} else {
		return sensorModel[d * modelSize[1] + a];
	}

}

//Represent the position of the tag relative to the robot in polar coordinates
void reduceToRobotSensorSystem(double robot_position_x,
		double robot_position_y, double robot_position_theta,
		double tag_position_x, double tag_position_y, int antenna_number,
		double* distance, double* angle) {

	// Antenna direction relative to -->  center:  robot position , direction: global x axis direction
	double thetaAntenna = ANTENNA_POSITION_ANGLES[antenna_number]
			+ robot_position_theta;

	//antenna absolute position
	double xa;
	double ya;

	xa = robot_position_x + ANTENNA_POSITION_RADIUS * cos(thetaAntenna);
	ya = robot_position_y + ANTENNA_POSITION_RADIUS * sin(thetaAntenna);

	*distance = sqrt((tag_position_x - xa) * (tag_position_x - xa)
			+ (tag_position_y - ya) * (tag_position_y - ya));

	// Tag angle relative to --> center: antenna position, direction: global x axis direction
	double thetaTag = atan2((tag_position_y - ya), (tag_position_x - xa));

	*angle = angleWrap(thetaTag - thetaAntenna);

}
