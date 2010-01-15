/*
 * RobotParticle.cc
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#include "RobotParticle.h"

RobotParticle::RobotParticle() {


}
RobotParticle::RobotParticle(double _x,double _y,double _theta ,double _weight )
{
	Particle(_x,_y,_weight);
	theta = _theta;
}

RobotParticle::~RobotParticle() {

}
