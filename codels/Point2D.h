/*
 * Point2D.h
 *
 *  Created on: 19 janv. 2010
 *      Author: andres
 */

#ifndef POINT2D_H_
#define POINT2D_H_

class Point2D {
public:
	double x;
	double y;
	Point2D();
	Point2D(double _x,double _y);
	virtual ~Point2D();
};

struct Point2DCmp {
	bool operator()(Point2D p1, Point2D p2) const {
		if(p1.x != p2 .x){
			return p1.x < p2.x;
		}else{
			return p1.y < p2.y;
		}
	}
};


#endif /* POINT2D_H_ */
