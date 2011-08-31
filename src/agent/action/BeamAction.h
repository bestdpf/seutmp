/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: BeamAction.h,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#ifndef ACTION_BEAN_ACTION_H
#define ACTION_BEAN_ACTION_H

#include "Action.h"
#include "math/Math.hpp"

namespace action {

class BeamAction : public Action
{
public:
    /** 
     * @param pos x, y and dirction
     */
    BeamAction(const math::Vector3f& pos);
	
	virtual ~BeamAction(){};
	
	virtual std::string command() const;
	
private:
	math::Vector3f mPos;
};

} // namespace action

#endif // ACTION_BEAN_ACTION_H
