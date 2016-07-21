#ifndef __COURSEMATH_H__
#define __COURSEMATH_H__

class PositionModel;

class CourseMath {
public:
	double calculateBTW(PositionModel boat, PositionModel waypoint) const;
	double calculateDTW(PositionModel boat, PositionModel waypoint) const;
};

#endif
