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


Tag::Tag(double _x, double _y, string _tagid){
	x = _x;
	y = _y;
	tagId = _tagid;
}

Tag::~Tag() {

}


