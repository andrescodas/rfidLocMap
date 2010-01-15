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

//a structure describing a point (x,y) in 2D
typedef struct Point2D {
	double x;
	double y;

	Point2D(double _x, double _y) {
		x = _x;
		y = _y;
	}

	Point2D() {
		x = 0;
		y = 0;
	}

} Point2D;

struct strCmp {
	bool operator()(const char* s1, const char* s2) const {
		return strcmp(s1, s2) < 0;
	}
};


//a map that allows us to recover a tag (position) by id
typedef std::map<const char*, Point2D, strCmp> TagMap;

TagMap initTagMap();


#endif /* TAG_H_ */
