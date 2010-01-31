/*
 * RobotParticle.cc
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#include "RobotParticle.h"
#include "stdio.h"


RobotParticle::RobotParticle() {


}
RobotParticle::RobotParticle(double _x,double _y,double _theta ,double _weight )
{
	x = _x;
	y = _y;
	weight = _weight;
	theta =  _theta;
}

void RobotParticle::print(){
	printf("Particle: x = %lf,\ty = %lf,\tt = %lf,\tw = %lf,\n",x,y,theta,weight);
}

RobotParticle::~RobotParticle() {

}
