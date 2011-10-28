
/***************************************************************************
*                              SEU RoboCup Simulation Team
*                     -------------------------------------------------
* Copyright (c) Southeast University , Nanjing, China.
* All rights reserved.
*
* $Id$
*
****************************************************************************/

#include "Task.h"
#include "KickTask.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"
#include "core/WorldModel.h"
#include "SwingFoot.h"
#include <fstream>
#include <sstream>

namespace task{

	using namespace boost;
	using namespace action;
	using namespace std;
	using namespace robot::humanoid;
	using namespace controller;
	using namespace perception;

DEFINE_STATIC_GRAPHIC_LOGGER(KickTask)


KickTask::KickTask( const string& firstTaskName, Task* primary ):
	Task(-1, primary )
{
	mIsFirstTime=true;
	static bool isFirstTime = true;
	mRaiseFootTask=firstTaskName;
	mDuration=FAT.calTaskTime(mRaiseFootTask) + 0.6;
	if(isFirstTime)
	{
		isFirstTime=false;
	}
}


shared_ptr<Action> KickTask::perform()
{
	if(mIsFirstTime)
	{
		mIsFirstTime=false;
		if(mStartTime<0)
		{
			mStartTime=WM.getSimTime();
		}

		shared_ptr<Action> act=Task::perform();

		if( NULL != act.get() )
		{
			//printf("%s\n","---return current task---");
			return act;
		}
		else	//start raise foot
		{
				FAT.setCurrentTask(mRaiseFootTask);
			
		}
	}


	string currentTaskName=FAT.currentTaskName();
	if( "null"==currentTaskName || "squat"==currentTaskName || "*"==currentTaskName ) /////////////// * ???
	{
		return FAT.controlPreferThan("squat", "*");
	}
	else
	{
		return FAT.controlPreferThan(currentTaskName,"*");
	}

} //end of KickTask::perform()
bool KickTask::isDone () const
{
	if(!mSubTaskList.empty()){
		return false;
	}

	LOG_PRINTF("isDone", "duration:%f,start time:%f,simTime:%f", mDuration,mStartTime,WM.getSimTime());
	LOG_FLUSH;
	return isTimeOut();
}



} //end of namespace task

