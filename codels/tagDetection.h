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

class TagDetection {

public:
	int antenna;
	char tagid[TAG_ID_MAX_SIZE + 1];
	TagDetection(int antenna, char* tagid);
	TagDetection();
	virtual ~TagDetection();
};

struct TagDetectionCompare {
	bool operator()(const TagDetection* t1, const TagDetection* t2) const {
		if (t1->antenna == t2->antenna) {
			return strcmp(t1->tagid, t2->tagid) < 0;
		}else{
			return (t1->antenna < t2->antenna);
		}
	}
};

typedef std::set< TagDetection*, TagDetectionCompare > TagDetectionSet;


void deleteTagDetections(TagDetectionSet* tagDetection);

#endif /* TAGDETECTION_H_ */
