/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: AgentSyncAction.cpp,v 1.0 2010/03/15  Allen Exp $
 *
 ****************************************************************************/

#include "AgentSyncAction.h"
#include <sstream>

namespace action {

std::string AgentSyncAction::command() const
{
	std::stringstream ss;
	ss<<"(syn )";
	return ss.str();
}

} // namespace action
