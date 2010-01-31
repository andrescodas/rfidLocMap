/*
 * tagDetection.cc
 *
 *  Created on: 13 janv. 2010
 *      Author: andres
 */

#include "tagDetection.h"
#include <string>
#include "stdio.h"
#include "stdlib.h"


TagDetection::TagDetection() {

}


TagDetection::TagDetection(int a, string t) {
	this->antenna = a;
	this->tagid = t;

}


void TagDetection::print(){
	printf("TagDetection: %s\t%d\n",this->tagid.c_str(),this->antenna);
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


void sortDetectionsByTagid(TagDetectionSet* inside, TagDetectionSet* tagDetectionSet, string tagid){
	TagDetectionSet::iterator tagDetectionIt ;

	for(tagDetectionIt = tagDetectionSet->begin();tagDetectionIt != tagDetectionSet->end();tagDetectionIt++){

		if((*tagDetectionIt)->tagid.compare(tagid) == 0){
			inside->insert(*tagDetectionIt);
		}
	}
}

void printDetectionSet(TagDetectionSet *tagDetectionSet){

	TagDetectionSet::iterator detectionIt;
	detectionIt = tagDetectionSet->begin();

	while(detectionIt != tagDetectionSet->end()){
		(*detectionIt)->print();
		detectionIt++;
	}
}
