/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "KeepBalance.h"
#include "GetUp.h"
#include "controller/FixedAngleTrace.h"
#include "core/WorldModel.h"
#include "configuration/Configuration.h"

namespace task {

using namespace boost;
using namespace action;
using namespace serversetting;

DEFINE_STATIC_GRAPHIC_LOGGER(KeepBalance)

KeepBalance::KBS KeepBalance::mCurrentState = BALANCE_STATE;
KeepBalance::KBS KeepBalance::mPossibleState = BALANCE_STATE;
KeepBalance::KBS KeepBalance::mLastPossibleState = BALANCE_STATE;
float KeepBalance::mStateKeepingStartTime = 0.0f;
float KeepBalance::unBalanceTime = 0.0f;

KeepBalance::KeepBalance():Task(-1, NULL)
{
	lieordive=IWANTTOLIE;
	LOG_PRINTF("keepBalance", "new KeepBalance and TaskLength= %d",mSubTaskList.size());

/*BEGIN_ADD_STATIC_LOG_LAYER(KeepBalance)
    ADD_LOG_LAYER("keepBalance");
    ADD_LOG_LAYER("whatToDo");
	ADD_LOG_LAYER("vision-me")
END_ADD_STATIC_LOG_LAYER(KeepBalance)*/
}

    void KeepBalance::reset()
    {
        clear();
        mPossibleState = BALANCE_STATE;
        mCurrentState = BALANCE_STATE;
        mLastPossibleState = BALANCE_STATE;
        mStateKeepingStartTime = WM.getSimTime();
        unBalanceTime = 0;
    }

KeepBalance::~KeepBalance()
{
}

void KeepBalance::updateState()
{
	//to do better, change value after the situation has changed 0.1secs.
	if ( WM.isLeftFall() ) {mPossibleState=LEFTFALL_STATE;LOG_PRINT("vision-me","11");}
	else if ( WM.isRightFall() ) {mPossibleState = RIGHTFALL_STATE;LOG_PRINT("vision-me","22");}
	else if ( WM.isDived() ) {mPossibleState = DIVED_STATE;LOG_PRINT("vision-me","33");}
	else if ( WM.isLied() ) {mPossibleState = LIED_STATE;LOG_PRINT("vision-me","44");}
	else if ( WM.isDiving() ) {mPossibleState = DIVING_STATE;LOG_PRINT("vision-me","55");}
	else if ( WM.isLying() ) {mPossibleState = LYING_STATE;LOG_PRINT("vision-me","66");}
	else{
        if ( mLastPossibleState == DIVED_STATE ||
             mLastPossibleState == LIED_STATE ||
             mLastPossibleState == BALANCE_STATE ){
            mPossibleState = BALANCE_STATE;
        }
    }

	if(WM.getMyAcc().z()>9.0f) mPossibleState=BALANCE_STATE;		//TT add

	if ( mPossibleState == BALANCE_STATE ) unBalanceTime = 0.0f;
	else unBalanceTime += 0.02f;

	LOG_PRINTF("keepBalance","mPossibleState is %d", mPossibleState);

	if ( mPossibleState == mLastPossibleState )
	{
		//LOG_PRINT("keepBalance","state continues");
		LOG_PRINTF("keepBalance","stateKeepingTime %f",WM.getSimTime()-mStateKeepingStartTime);
	}
	else
	{
		mStateKeepingStartTime = WM.getSimTime();	
		LOG_PRINTF("keepBalance","stateKeepingTime %f",WM.getSimTime()-mStateKeepingStartTime);
	}

	mLastPossibleState = mPossibleState;	
	mCurrentState = ( WM.getSimTime()-mStateKeepingStartTime > 0.09f ) ? mPossibleState : mCurrentState; //根据状态持续时间判断是否需要更新当前状态

	LOG_PRINTF("keepBalance","mCurrentState is %d",mCurrentState);
}

void KeepBalance::analysisWhatToDo()
{
	shared_ptr<Task> getUp;
	shared_ptr<Task> bufferAct;

	if(mDuration>2.0f)
		{
			if(lieordive==IWANTTODIVE) lieordive = IWANTTOLIE;
			else	lieordive = IWANTTODIVE;
		}

    float stateKeepingTime = WM.getSimTime() - mStateKeepingStartTime;

	if(mCurrentState == LEFTFALL_STATE && stateKeepingTime >0.3f ){
		if(lieordive==IWANTTOLIE) getUp = shared_ptr<Task> (new LeftFallToLie(this));
		else	getUp = shared_ptr<Task> (new LeftFallToDive(this));
		LOG_PRINTF("whatToDo","LeftFallToLie keeptime=%.3f",stateKeepingTime);
	}
	else if( mCurrentState == RIGHTFALL_STATE && stateKeepingTime > 0.3f ){
		if(lieordive==IWANTTOLIE)	getUp = shared_ptr<Task> (new RightFallToLie(this));
		else	getUp = shared_ptr<Task> (new RightFallToDive(this));
		LOG_PRINTF("whatToDo","RightFallToLie keeptime=%.3f",stateKeepingTime);
	}
	else if ( mCurrentState == LIED_STATE && stateKeepingTime > 0.3f ){
        // if it is timeout.
// 		cerr<<"GetUpFromLie"<<endl;
		getUp = shared_ptr<Task> (new GetUpFromLie(this));// if the situation is 0.3 secs long.
        LOG_PRINTF("whatToDo","GetUpFromLie keeptime=%.3f",stateKeepingTime);
    }
	else if ( mCurrentState == DIVED_STATE && stateKeepingTime > 0.3f ){
// 		cerr<<"GetUpFromDive"<<endl;
        getUp = shared_ptr<Task> ( new GetUpFromDive(this) );
        LOG_PRINTF("whatToDo","GetUpFromDive keepTime=%.3f",stateKeepingTime);
    }
	if ( 0 != getUp.get() )
	{
		stop();
		mSubTaskList.push_back(getUp);
		LOG_PRINT("whatToDo","getUp Action is not null");
	}
	else
	{
		LOG_PRINT("whatToDo","getUp Action is null");
	}
}

shared_ptr<action::Action> KeepBalance::perform()
{
	updateState();		

#ifdef ENABLE_LOG
	float leftLegHeight = 0;// TODO WM.getJointTrans(JID_LEG_L_4).p().z();
	float rightLegHeight = 0;//TODO WM.getJointTrans(JID_LEG_R_4).p().z();
	float leftArmHeight = 0;//TODO WM.getJointTrans(JID_ARM_L_4).p().z();
	float rightArmHeight = 0;//TODO WM.getJointTrans(JID_ARM_R_4).p().z();

	LOG_PRINTF("keepBalance","leftLeg %f, rightLeg %f, leftArm %f, rightArm %f", leftLegHeight, rightLegHeight, leftArmHeight, rightArmHeight);
#endif

	if (isDone() || isTerminable())
	{
		if (isDone())
		{
			LOG_PRINT("keepBalance","isDone");
		}
		else
		{
			LOG_PRINT("keepBalance","isTerminable");
		}
		analysisWhatToDo();
	}
	else
	{
		LOG_PRINT("keepBalance","is not done and not terminable");
	}

	LOG_FLUSH
	return Task::perform();
}

}

