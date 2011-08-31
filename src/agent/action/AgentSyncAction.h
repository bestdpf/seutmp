/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: AgentSyncAction.h,v 1.0 2010/03/15  Allen Exp $
 *
 ****************************************************************************/

#ifndef ACTION_AGENTSYNC_ACTION_H
#define ACTION_AGENTSYNC_ACTION_H

#include "Action.h"
#include "math/Math.hpp"

namespace action {

class AgentSyncAction : public Action
{
public:
    /**
     * @param pos x, y and dirction
     */
    AgentSyncAction(){};

	virtual ~AgentSyncAction(){};

	virtual std::string command() const;

private:
	
};

} // namespace action

#endif // ACTION_AGENTSYNC_ACTION_H
