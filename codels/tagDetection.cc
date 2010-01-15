/*
 * tagDetection.cc
 *
 *  Created on: 13 janv. 2010
 *      Author: andres
 */

#include "tagDetection.h"
#include <string>

TagDetection::TagDetection() {

}


TagDetection::TagDetection(int a, char* t) {
	this->antenna = a;
	strcpy(this->tagid , t );

}


TagDetection::~TagDetection() {


}

void deleteTagDetections(TagDetectionSet* tagDetectionSet){

	TagDetectionSet::iterator tagDetectionIt;
	tagDetectionIt = (*tagDetectionSet).begin();
	while (tagDetectionIt != (*tagDetectionSet).end()){
		delete (*tagDetectionIt);
		tagDetectionIt++;
	}


}

