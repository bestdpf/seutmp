
/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef TASK_KICKTASK_H
#define TASK_KICKTASK_H

#define ENABLE_TASK_KICKTASK_LOG

#include "Task.h"
#ifdef ENABLE_TASK_KICKTASK_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{

using namespace std;

class KickTask: public Task
{
public:
	KickTask( const string& firstTaskName, Task* primary=NULL );

	virtual boost::shared_ptr<action::Action> perform();
	bool isDone () const;

private:
	///////////////////////////////////// TT test
	bool mIsFirstTime;
	////////////////////
	std::string mRaiseFootTask;

	DECLARE_STATIC_GRAPHIC_LOGGER;
};


}

#endif // TASK_KICKTASK_H

