/*
 * geometricalTools.cc
 *
 *  Created on: 15 janv. 2010
 *      Author: andres
 */

#include "geometricalTools.h"

// force angle representation to ( -PI,PI]
double angleWrap(double angle) {
	double angleAux = angle;

	while (angleAux <= -PI) {
		angleAux = angleAux + 2 * PI;
	}

	while (angleAux > PI) {
		angleAux = angleAux - 2 * PI;
	}

	return angleAux;
}
