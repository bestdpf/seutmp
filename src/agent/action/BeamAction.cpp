/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: BeamAction.cpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#include "BeamAction.h"
#include <sstream>

namespace action {

BeamAction::BeamAction(const math::Vector3f& pos)
:mPos(pos)
{
}

std::string BeamAction::command() const
{
	std::stringstream ss;
	ss<<"(beam "<<mPos<<')';
	return ss.str();
}

} // namespace action
