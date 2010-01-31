/*
 * tagDetection.h
 *
 *  Created on: 13 janv. 2010
 *      Author: andres
 */

#ifndef TAGDETECTION_H_
#define TAGDETECTION_H_

#include "string.h"
#include "Tag.h"
#include <set>
#include "TagParticles.h"

using namespace std;

class TagDetection {

public:
	int antenna;
	string tagid;
	TagDetection(int antenna, string tagid);
	TagDetection();
	void print();
	virtual ~TagDetection();
};

struct TagDetectionCompare {
	bool operator()(const TagDetection* t1, const TagDetection* t2) const {
		if (t1->antenna == t2->antenna) {
			return t1->tagid.compare(t2->tagid) < 0;
		}else{
			return (t1->antenna < t2->antenna);
		}
	}
};

typedef std::set< TagDetection*, TagDetectionCompare > TagDetectionSet;

void printDetectionSet(TagDetectionSet *tagDetectionSet);

void sortDetectionsByTagMap(TagDetectionSet* inside,TagDetectionSet* outside, TagDetectionSet* tagDetectionSet, TagMap *tagMap);

void sortDetectionsByTagParticlesMap(TagDetectionSet* inside,TagDetectionSet* outside, TagDetectionSet* tagDetectionSet, TagParticlesMap *tagParticlesMap);

void sortDetectionsByTagid(TagDetectionSet* inside, TagDetectionSet* tagDetectionSet, string tagid);

void deleteTagDetections(TagDetectionSet* tagDetection);

#endif /* TAGDETECTION_H_ */
