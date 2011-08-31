/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/


#ifndef PERCEPTION_JOINT_H
#define PERCEPTION_JOINT_H

#include "math/Math.hpp"
#include "BasicPerception.h"
#include <iostream>

namespace perception {

class Joint: public BasicPerception
{
public:
    Joint(math::AngDeg a=0, math::AngDeg r=0):mAngle(a),mRate(r){};

    Joint(const sexp_t* sexp);
    
	Joint(const sexp_t* sexpAx, const sexp_t* sexpRt);
	
	~Joint(){};
	
	virtual bool update(const sexp_t* sexp);
    
    bool update(const sexp_t* sexpAx, const sexp_t* sexpRt);
		
	void setAngle(math::AngDeg ang) { mAngle = ang; }
	
	void setRate(math::AngDeg rate) { mRate = rate; }
	
	friend std::ostream& operator<<(std::ostream &stream, const Joint& j);

    math::AngDeg angle() const { return mAngle; }
	
    math::AngDeg rate() const { return mRate; }
	
	void angReverse() { mAngle = -mAngle; }
	
	Joint& operator+=(math::AngDeg ang);
	
private:
    math::AngDeg mAngle, mRate;
};

std::ostream& operator<<(std::ostream &stream, const Joint& j);

} // namespace perception

#endif // PERCEPTION_JOINT_H
