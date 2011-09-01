/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Actions.h"
//#include<stdio.h>/////////////////////////////////////////////////////

namespace action{

Actions::Actions()
{
}

Actions::~Actions()
{
}

std::string Actions::command() const
{
	std::string cmds;
	for(TActionPtrs::const_iterator iter=mActs.begin(); iter!=mActs.end(); ++iter){
		cmds += (*iter)->command();
	}
	return cmds;
}

void Actions::add( boost::shared_ptr<Action> act )
{
    if ( 0 != act.get() ){
        mActs.push_back(act);
    }
}


} //end of namespace action

