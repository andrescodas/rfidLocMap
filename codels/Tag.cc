/*
 * Tag.cc
 *
 *  Created on: 14 janv. 2010
 *      Author: andres
 */

#include "Tag.h"
#include "string.h"

Tag::Tag() {


}


Tag::Tag(double _x, double _y, const char* _tagid){
	x = _x;
	y = _y;
	for (unsigned int i = 0; i < strlen(_tagid); ++i) {
		tagid[i] = _tagid[i];
	}
	tagid[TAG_ID_MAX_SIZE] = 0;
}

Tag::~Tag() {

}

//initTagMap
//  - description
//      - the function allows the construction of the list of tags (all tags in the environment should be inputed by modifying this function)
//  - input: none
//  - output: the tag map
TagMap initTagMap() {
	TagMap tagmap;
	tagmap.clear();

	tagmap["e0040000c1b2fd01\0"] = Point2D(-4.07, -1); //tag: 0
	tagmap["e00400007cd7fc01\0"] = Point2D(-4.07, 1.84); //tag: 1
	tagmap["e0040000079efd01\0"] = Point2D(3, 1.5); //tag: 2
	tagmap["e004000076defc01\0"] = Point2D(00.50, 3.55); //tag: 3
	tagmap["e0040000dab1fd01\0"] = Point2D(01.50, +00.50); //tag: 4
	tagmap["e0040000fa9afd01\0"] = Point2D(+02.50, -03.00); //tag: 5
	tagmap["e004000080ddfc01\0"] = Point2D(-0.84, -03.27); //tag: 6


	return tagmap;
}

