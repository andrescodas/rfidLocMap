/*
 * Tag.h
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#ifndef TAG_H_
#define TAG_H_

#include <map>
#include <string.h>
#include "Point2D.h"

const int TAG_ID_MAX_SIZE = 16;


class Tag {
public:
	double x;
	double y;
	char tagid[TAG_ID_MAX_SIZE + 1];
	Tag(double _x, double _y, const char* _tagid);
	Tag();
	virtual ~Tag();
};


struct strCmp {
	bool operator()(const char* s1, const char* s2) const {
		return strcmp(s1, s2) < 0;
	}
};


//a map that allows us to recover a tag (position) by id
typedef std::map<const char*, Point2D, strCmp> TagMap;

TagMap initTagMap();


#endif /* TAG_H_ */
