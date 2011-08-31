
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

#include "BasicKick.h"
#ifdef ENABLE_TASK_KICKTASK_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{

using namespace std;

class KickTask: public BasicKick
{
public:
	KickTask( const string& firstTaskName, Task* primary=NULL );

	virtual boost::shared_ptr<action::Action> perform();


private:

	virtual void generateAnkleDistTable();

	virtual void generateDurationDistTable();

	virtual AngDeg getDesiredAnkleAngle();

	virtual float getDesiredDuration();

private:
	float mAcc2Duration;
	bool mMaxForce;

	///////////////////////////////////// TT test
	bool mIsFirstTime;
	////////////////////

	DECLARE_STATIC_GRAPHIC_LOGGER;
};


}

#endif // TASK_KICKTASK_H

