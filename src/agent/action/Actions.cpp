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
/*
	int i=0;
	for(TActionPtrs::const_iterator iter1=mActs.begin(); iter1!=mActs.end(); ++iter1)
	{
		printf("i=%d\n%s\n",i,(*iter1)->command().c_str());
		i++;
	}
	printf("\n==========================\n");
*/
	/////////////////////////////////////////////////////// TT test
	std::string cmds;
	for(TActionPtrs::const_iterator iter=mActs.begin(); iter!=mActs.end(); ++iter){
		cmds += (*iter)->command();
	}
	//printf("%s\n",cmds.c_str());
	return cmds;
}

void Actions::add( boost::shared_ptr<Action> act )
{
    if ( 0 != act.get() ){
        mActs.push_back(act);
    }
}


} //end of namespace action

