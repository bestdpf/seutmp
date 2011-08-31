/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WalkRel.cpp 2011-03-07 TT $
 *
 ****************************************************************************/

#include <list>
#include "core/WorldModel.h"
#include "configuration/Configuration.h"
//#include "MoveCoM.h"
//#include "MoveFoot.h"
#include "Step.h"
#include "WalkRel.h"


namespace task{

using namespace std;
using namespace boost;
using namespace math;
using namespace serversetting;
using namespace action;


float WalkRel::mWalkHeight = 0.35f;

DEFINE_STATIC_GRAPHIC_LOGGER(WalkRel)


WalkRel::WalkRel(const Vector2f& target, AngDeg direction, bool avoidBall, Task* primary)
	:Task(-1, primary), // -1: as soon as possible
	mTarget(target),
	mDirection(direction),
	mSizeErrorThreshold(0.01f,0.01f),
	mDirErrorThreshold(3),
	mIsWalking(false),
	mAvoidBall(avoidBall)
{
	BEGIN_ADD_STATIC_LOG_LAYER(WalkRel)
		ADD_LOG_LAYER("newWalk")
		ADD_LOG_LAYER("newStep")
		ADD_LOG_LAYER("planPath")
		ADD_LOG_LAYER("blocks")
	END_ADD_STATIC_LOG_LAYER(WalkRel)

	mPreSize.zero();
	mShouldStop=false;
}


bool WalkRel::isDone() const
{
	if(!mIsWalking)
		return false;

	return mSubTaskList.empty();
}


bool WalkRel::revise( shared_ptr<const Task> rt )
{
	///////////////////////////////// test
	//printf("WalkRel: size= %d\n",getSubTaskListSize());
	/////////////////////////////////

	shared_ptr<const WalkRel> wrt=shared_dynamic_cast<const WalkRel>(rt);
	if( NULL != wrt.get() )
	{
		// accept another WalkRel
		mTarget = wrt->mTarget;
		mDirection = wrt->mDirection;
		mSizeErrorThreshold = wrt->mSizeErrorThreshold;
		mDirErrorThreshold = wrt->mDirErrorThreshold;
		mAvoidBall = wrt->mAvoidBall;
		return true;
	}

	else
		return false;
}


void WalkRel::updateSubTaskList()
{
	//if it's too early to calculate next step, then return
	Task::updateSubTaskList();
	if( false==isSubTaskOfAllLessThanTwo() )
		return;

	//...
	mIsWalking=true;

	//...
	bool isLeft=true;
	AngDeg preDir=0;
	shared_ptr<const Step> cStep; //current step
	if( !mSubTaskList.empty() )
	{
		cStep=shared_dynamic_cast<const Step>( mSubTaskList.front() ); //get the last step
		if( 0!=cStep.get() )
		{
			isLeft= !cStep->isLeft(); //change foot
			preDir=cStep->dir();
			mPreSize=cStep->size();
		}
	}

	//TT, for Walk and WalkRel
	if( mShouldStop /*&& mPreSize.length()<0.01f*/ )
		return;

	//new step
	Vector2f size=mTarget*0.5; //NOTE: the real step size is double "size"

	//judge if it's necessary to add a new step
	/*const Vector3f& bodyAcc=WM.getMyAcc();//////////////////////////////////////////////////////
	if( fabs(size[0])+fabs(size[1])<0.012f &&//0.01f
		fabs(mPreSize[0])+fabs(mPreSize[1])<0.012f &&//0.01f
		fabs(mDirection)<8.0f &&
		fabs(bodyAcc.y())<2.0f &&
		fabs(bodyAcc.x())<1.5f )
	{
		return;///////////////////////////////////////////////////////////////////////////////////
	}*/

	shared_ptr<Task> mstep( new Step(isLeft,
									 size,
									 mDirection,
									 cStep,
									 0.35f,
									 this) );
	mSubTaskList.push_back(mstep);
}


} //end of namespace task

