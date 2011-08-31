/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: InitAction.h,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#ifndef ACTION_INIT_ACTION_H
#define ACTION_INIT_ACTION_H

#include "Action.h"

namespace action {

class InitAction : public Action
{
public:

    InitAction(const std::string& teamname, unsigned int unum = 0);
	
	virtual ~InitAction(){};
	
	virtual std::string command() const;
	
private:
    std::string mTeamname;
	unsigned int mUnum;
};

} // namespace action

#endif // ACTION_INIT_ACTION_H
