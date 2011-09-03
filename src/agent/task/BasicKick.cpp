/***************************************************************************
*                              SEU RoboCup Simulation Team
*                     -------------------------------------------------
* Copyright (c) Southeast University , Nanjing, China.
* All rights reserved.
*
* $Id:$
*
****************************************************************************/

#include "BasicKick.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"
#include "Walk.h"
#include "MoveCoM.h"
#include "perception/JointPerception.h"
#include "core/WorldModel.h"

namespace task
{

using namespace boost;
using namespace action;
using namespace perception;
using namespace std;
using namespace robot::humanoid;
using namespace controller;

BasicKick::TAnkleDist BasicKick::mAnkleDist;
BasicKick::TDurationDist BasicKick::mDurationDist;

DEFINE_STATIC_GRAPHIC_LOGGER(BasicKick)

BasicKick::BasicKick( const math::Vector2f& target, const math::Vector2f& movePos, const float biasAngle, bool isLeftLeg,Task* primary ):
Task( -1, primary ), mMovePos(movePos), mIsLeftLeg(isLeftLeg), mBiasAngle(biasAngle)
{
	mTarget = target;
	mMoveCoMDuration = 0.4f;
	mAccStartTime = -1;
	mBaituiStartTime = -1;
	mAccJointAngles.clear();
	mBaituiJointAngles.clear();
	mAmendDuration = -1;
	mSquatDuration = 0.6f;
	mSwingFootHoldTime = 0.04f;
	mBallPos2D = Vector2f(999.0f,999.0f);
}

shared_ptr<Action> BasicKick::perform()
{
	LOG_PRINTF("desiredDuration", "mSubTaskList.size=%d", mSubTaskList.size());
	LOG_FLUSH;
	shared_ptr<Action> act;
	if(mSubTaskList.size() > 0)
	{
		if(-1 == mStartTime)
		{
			mStartTime = WM.getSimTime();
			mDuration = FAT.calTaskTime(mRaiseRFootTask) + mMoveCoMDuration * 2;
		}

		LOG_PRINT("desiredDuration", "MoveCoM");
		act = Task::perform();

		if(NULL != act.get())
		{
			LOG_FLUSH;
			return act;
		}
		else	
		{
			FAT.setCurrentTask(mRaiseRFootTask);
		}
	}

	string currentTaskName = FAT.currentTaskName();
	LOG_PRINTF("desiredDuration", "currentTaskName:%s", currentTaskName.c_str());

	if(currentTaskName == mRaiseRFootTask)
	{
		LOG_FLUSH;
		return FAT.controlPreferThan(mRaiseRFootTask, "*");
	}
	else
	{
		LOG_PRINT("desiredDuration","squat");
		LOG_FLUSH;
		return FAT.controlPreferThan("squat", "*");
	}
}


bool BasicKick::isAchieveable () const
{
	const Vector2f& myPos2D = WM.getMyOrigin2D();
	const Vector2f& target = mKickParameter.movePos;
	Vector2f err = myPos2D - target;
	AngDeg currentAng = WM.getMyBodyDirection();
	return(abs(err.x()) < 0.05) && (abs(err.y()) < 0.05)
	&& (abs(calClipAng(mKickParameter.bodyAng, currentAng)) < 5);
}

bool BasicKick::isDone () const
{
	if(!mSubTaskList.empty()){
		return false;
	}

	LOG_PRINTF("isDone", "duration:%f,start time:%f,simTime:%f", mDuration,mStartTime,WM.getSimTime());
	LOG_FLUSH;
	return isTimeOut();
}


} //end of namespace task

