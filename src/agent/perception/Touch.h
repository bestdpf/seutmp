/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Touch.h,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#ifndef PERCEPTION_TOUCH_H
#define PERCEPTION_TOUCH_H

#include "math/Math.hpp"
#include "BasicPerception.h"


namespace perception {
	
class Touch: public BasicPerception
{
public:
	Touch();
	~Touch();
	
	virtual bool update(const sexp_t* sexp);

	friend std::ostream& operator<<(std::ostream &stream, const Touch& t);

	typedef std::map<unsigned int, int> TValMap;

	int val(unsigned int id) const { return mVals.find(id)->second; }

private:
	TValMap mVals;
};

std::ostream& operator<<(std::ostream &stream, const Touch& t);

} // namespace perception

#endif // PERCEPTION_TOUCH_H
