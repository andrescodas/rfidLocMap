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


void sortDetectionsByTagMap(TagDetectionSet* inside,TagDetectionSet* outside, TagDetectionSet* tagDetectionSet, TagMap *tagMap){
	TagDetectionSet::iterator tagDetectionIt ;
	TagMap::iterator tagIt ;

	for(tagDetectionIt = tagDetectionSet->begin();tagDetectionIt != tagDetectionSet->end();tagDetectionIt++){
		tagIt = tagMap->find((*tagDetectionIt)->tagid);
		if(tagIt != tagMap->end()){
			inside->insert(*tagDetectionIt);
		}else{
			outside->insert(*tagDetectionIt);
		}
	}
}

void sortDetectionsByTagParticlesMap(TagDetectionSet* inside,TagDetectionSet* outside, TagDetectionSet* tagDetectionSet, TagParticlesMap *tagParticlesMap){
	TagDetectionSet::iterator tagDetectionIt ;
	TagParticlesMap::iterator tagIt ;

	for(tagDetectionIt = tagDetectionSet->begin();tagDetectionIt != tagDetectionSet->end();tagDetectionIt++){
		tagIt = tagParticlesMap->find((*tagDetectionIt)->tagid);
		if(tagIt != tagParticlesMap->end()){
			inside->insert(*tagDetectionIt);
		}else{
			outside->insert(*tagDetectionIt);
		}
	}
}


void sortDetectionsByTagid(TagDetectionSet* inside, TagDetectionSet* tagDetectionSet, const char* tagid){
	TagDetectionSet::iterator tagDetectionIt ;

	for(tagDetectionIt = tagDetectionSet->begin();tagDetectionIt != tagDetectionSet->end();tagDetectionIt++){

		if(strcmp((*tagDetectionIt)->tagid,tagid) == 0){
			inside->insert(*tagDetectionIt);
		}
	}
}
