/*
 * Tag.h
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#ifndef TAG_H_
#define TAG_H_

#include <map>
#include <string>
#include "Point2D.h"

using namespace std;

const int TAG_ID_MAX_SIZE = 16;


class Tag {
public:
	double x;
	double y;
	string tagId;
	Tag(double _x, double _y, string _tagid);
	Tag();
	virtual ~Tag();
};

/*
struct strCmp {
	bool operator()( string s1, string s2) const {
		return (s1.compare(s2) == 0);
	}
};
*/

//a map that allows us to recover a tag (position) by id
typedef map<string , Point2D> TagMap;


#endif /* TAG_H_ */
