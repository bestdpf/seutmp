/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: InitAction.cpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#include "InitAction.h"
#include <sstream>

namespace action {

InitAction::InitAction(const std::string& teamname, unsigned int unum)
:mTeamname(teamname),mUnum(unum)
{
}

std::string InitAction::command() const
{
	std::stringstream ss;
	ss<<"(init (unum "<<mUnum<<")(teamname "<<mTeamname<<"))";
	return ss.str();
}


} // namespace action
