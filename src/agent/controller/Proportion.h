/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef CONTROLLER_PROPRTION_H
#define CONTROLLER_PROPRTION_H

#include "../perception/JointPerception.h"
#include "../action/JointAction.h"
#include <boost/shared_ptr.hpp>

namespace controller {

class Proportion
{
public:
	Proportion(float k=1):mK(k){};
	
	~Proportion(){};
	
	void setK(float k) { mK = k; }
	
	boost::shared_ptr<action::Action>
	control(const perception::JointPerception& p0, const perception::JointPerception& p1);
	
private:
	float mK;
};

} // namespace controller

#endif // CONTROLLER_PROPRTION_H
