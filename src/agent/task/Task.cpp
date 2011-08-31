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
#include "core/WorldModel.h"

namespace task{

using namespace boost;
using namespace action;

Task::Task(float time, Task* primary)
	:mDuration(time),
	 mStartTime(-1),
	 mParentTask(primary)
{
}

bool Task::isAchieveable() const
{
	if ( mSubTaskList.empty() ){
		// no sub task
		// assume we can stark work in every state,
		// specially defination shoule be in dervied class
		return true;
	}

	// see the first sub task
	return mSubTaskList.front()->isAchieveable();
}

bool Task::isTerminable() const
{
	if ( mSubTaskList.empty() ){
		// no sub task
		// assume we can stop work in every state,
		// specially defination shoule be in dervied class
		return true;
	}

	// see the first sub task
	return mSubTaskList.front()->isTerminable();
}

bool Task::isDone() const
{
	if ( mSubTaskList.empty() ) return true;

	if ( 1 == mSubTaskList.size() )
		return mSubTaskList.front()->isDone();

	return false;
}


shared_ptr<Action> Task::perform()
{
	if( mStartTime < 0 ){
		// first perform
		mStartTime = WM.getSimTime();
	}

	updateSubTaskList();

	if( mSubTaskList.empty() ){
		// done
		return shared_ptr<Action>();
	}

	return mSubTaskList.front()->perform();
}


void Task::updateSubTaskList()
{
	if( !mSubTaskList.empty() )
	{
		if( mSubTaskList.front()->isDone() ){
			mSubTaskList.pop_front();
		}
		/*else{
			mSubTaskList.front()->updateSubTaskList();
		}*/
	}
}


bool Task::stop()
{
	while( !mSubTaskList.empty() ){
		shared_ptr<Task> sub = mSubTaskList.front();
		if ( !sub->isAchieveable() || sub->isTerminable() ){
			mSubTaskList.pop_front();
		}
		else{
			return false;
		}
	}
	return true;
}

bool Task::isSubDone() const
{
	shared_ptr<const Task> first = getFirstOfAllSubTask();

	if ( 0 == first.get() ){
		// no sub task
		return isDone();
	}
	return first->isDone();
}

shared_ptr<Task> Task::getFirstSubTask()
{
	if ( mSubTaskList.empty() ){
		return shared_ptr<Task>();
	}
	else{
		return mSubTaskList.front();
	}
}

shared_ptr<const Task> Task::getFirstSubTask() const
{
	if ( mSubTaskList.empty() ){
		return shared_ptr<const Task>();
	}
	else{
		return mSubTaskList.front();
	}
}


shared_ptr<Task> Task::getSecondSubTask()
{
	TTaskList::iterator iter =  mSubTaskList.begin();
	// the sub-task list is empty
	if ( mSubTaskList.end() == iter ) return shared_ptr<Task>();
	iter++;
	// ther is only one sub task
	if ( mSubTaskList.end() == iter ) return shared_ptr<Task>();
	return *iter;
}


shared_ptr<const Task> Task::getSecondSubTask() const
{
	TTaskList::const_iterator iter =  mSubTaskList.begin();
	// the sub-task list is empty
	if ( mSubTaskList.end() == iter ) return shared_ptr<const Task>();
	iter++;
	// ther is only one sub task
	if ( mSubTaskList.end() == iter ) return shared_ptr<const Task>();
	return *iter;
}

shared_ptr<const Task> Task::getFirstOfAllSubTask() const
{
	if ( mSubTaskList.empty() ){
		return shared_ptr<const Task>();
	}

	shared_ptr<const Task> firstSub = mSubTaskList.front();
	shared_ptr<const Task> firstSubAll = firstSub->getFirstOfAllSubTask();
	if ( 0 == firstSubAll.get() ) return firstSub;
	return firstSubAll;
}

shared_ptr<const Task> Task::getNextOfAllSubTask() const
{
	// it is the primariest task
	if ( isTop() ) return shared_ptr<const Task>();

	shared_ptr<const Task> pSecond = mParentTask->getSecondSubTask();
	if ( 0 == pSecond.get() ){
		// there is only one sub-task in parent's list
		return mParentTask->getNextOfAllSubTask();
	}

	shared_ptr<const Task> pFirstOfSec = pSecond->getFirstOfAllSubTask();
	if ( 0 == pFirstOfSec.get() ) return pSecond;
	return pFirstOfSec;
}

bool Task::isSubTaskOfAllLessThanTwo() const
{
	if ( mSubTaskList.empty() ) return true;
	if ( mSubTaskList.size() > 1 ) return false;

	return mSubTaskList.front()->isSubTaskOfAllLessThanTwo();
}

bool Task::isSubTaskOfAllLessThanThree() const								/////terrymimi
{
	if ( mSubTaskList.empty() ) return true;
	if ( mSubTaskList.size() > 2 ) return false;

	return mSubTaskList.front()->isSubTaskOfAllLessThanThree();
}

float Task::getRemainTime() const
{
	//printf("mStartTime=%f\tmDuration=%f\n",mStartTime,mDuration);
	if ( mStartTime < 0 ){
		// not start yet
		return mDuration;
	}
	else{
		return mDuration + mStartTime - WM.getSimTime();
	}
}

bool Task::isFirstDoAction() const
{
	if(fabs(mStartTime - WM.getSimTime()) < 0.0001)
		return true;
	return false;
}


} //end of namespace task

