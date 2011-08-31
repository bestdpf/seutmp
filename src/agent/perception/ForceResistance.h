/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
 
#ifndef PERCEPTION_FORCE_RESISTANCE_H
#define PERCEPTION_FORCE_RESISTANCE_H

#include "math/Math.hpp"
#include "BasicPerception.h"


namespace perception {
	
class ForceResistance: public BasicPerception
{
public:
	ForceResistance();
	~ForceResistance();
	
	virtual bool update(const sexp_t* sexp);

	friend std::ostream& operator<<(std::ostream &stream, const ForceResistance& fr);

	struct FeedBack
	{
        math::Vector3f pos; // the center position of force feed back
        math::Vector3f force; // the force of force feed back
	};
	typedef std::map<unsigned int, FeedBack > TValMap;

	//const Vector3f& pos(serversetting::ForceResistanceID id) const { return mVals.find(id)->second; }
	
	bool isTouch(unsigned int id) const
        { return mVals.find(id) != mVals.end(); }
	
	bool isDoubleSupport() const { return mVals.size() > 1; }
	
	const FeedBack& feedBack(unsigned int id) const
        { return mVals.find(id)->second; }
	
private:
	TValMap mVals;
};

std::ostream& operator<<(std::ostream &stream, const ForceResistance& fr);

} // namespace perception

#endif // PERCEPTION_FORCE_RESISTANCE_H
