/***************************************************************************
*                              SEU RoboCup Simulation Team
*                     -------------------------------------------------
* Copyright (c) Southeast University , Nanjing, China.
* All rights reserved.
*
* $Id$
*
****************************************************************************/

#include "Shoot.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"
#include "Walk.h"
#include "MoveCoM.h"
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

DEFINE_STATIC_GRAPHIC_LOGGER(Shoot)


Shoot::Shoot( const math::Vector2f& target, bool isLeftLeg, bool maxForce, Task* primary ):
	BasicKick( target, Vector2f(-0.14, 0.05), 2.0, isLeftLeg, primary)
{
	mIsFirstTime=true;////////////////////////// TT test
	static bool isFirstTime = true;
	//mRaiseRFootTask = "hu_kick2R";
	//mRaiseRFootTask="oblique45IIR";/////////////////////////////// TT test
	mRaiseRFootTask="ATT_t1";
        mDuration=FAT.calTaskTime(mRaiseRFootTask) + mSquatDuration;
	//mDuration=FAT.calTaskTime(mRaiseRFootTask) + mMoveCoMDuration*2 + mSquatDuration;/////////////////////// TT test OK
mRaiseLFootTask = "BTT_t1";
        //	mRaiseLFootTask = "hu_kick2L";
//	mBaiRtuiTask = "hu_kick3R";
//	mBaiLtuiTask = "hu_kick3L";
//	mAccRTask = "hu_kick4R";
//	mAccLTask = "hu_kick4L";
//        		mRaiseRFootTask = "sidekick1R";
//		mRaiseLFootTask = "sidekick1L";
//		mBaiRtuiTask = "sidekick2";
//		mAccRTask = "sidekick3";
	mAcc2Duration = 0.04;
	mMaxForce = maxForce;

	if(isFirstTime)
	{
		isFirstTime=false;
		generateAnkleDistTable();
		generateDurationDistTable();
	}

	/*BEGIN_ADD_STATIC_LOG_LAYER(Shoot)
	ADD_LOG_LAYER("supportfoot");
	ADD_LOG_LAYER("DesiredTorso");
	ADD_LOG_LAYER("terrymimi");
	ADD_LOG_LAYER("dura");
	END_ADD_STATIC_LOG_LAYER(Shoot);*/
}


AngDeg Shoot::getDesiredAnkleAngle()
{
	const Vector2f& posBall2D = WM.getInterceptBallGlobalPos2D();
	float dist = (posBall2D-mTarget).length();
	float err = 999;
	AngDeg ankle = 0;
	for( TAnkleDist::const_iterator iter = mAnkleDist.begin(); iter !=mAnkleDist.end(); iter ++ )
	{
		if( abs(iter->second - dist) < err )	// The current angle can make the ball closer to the target.
		{
			err = abs(iter->second - dist);
			ankle = iter->first;
		}
	}

	return ankle;
}


float Shoot::getDesiredDuration()
{
	const Vector2f posBall2D = WM.getBallGlobalPos2D();
	float dist = (posBall2D-mTarget).length();
	// float err = 999;
	float duration = 0;

	if( dist >= 3.0f || mMaxForce )
		duration = 0.1f;
	else
		duration = 0.2f;

	LOG_PRINTF("KickParameter", "acc duration:%f", duration);
	return duration;
}


void Shoot::generateAnkleDistTable()
{
	ifstream data;
	data.open("data/shoot_rankle-dist.data");

	if(data.is_open())
	{
		while(!data.eof())
		{
			char l[30];
			AngDeg ankle;
			Vector2f finalBallPos;
			data.getline(l, 30);
			data.get();	// read enter
			stringstream ss(l);
			ss >> ankle >> finalBallPos[0] >> finalBallPos[1];
			mAnkleDist[ankle] = Vector2f(finalBallPos[0], finalBallPos[1]).length();
		}
	}
	data.close();
}


void Shoot::generateDurationDistTable()
{
	ifstream data;
	data.open("data/shoot_duration-dist.data");

	if(data.is_open())
	{
		while(!data.eof())
		{
			char l[30];
			float duration;
			Vector2f finalBallPos;
			data.getline(l, 30);
			data.get();	// read enter
			stringstream ss(l);
			ss >> duration >> finalBallPos[0] >> finalBallPos[1];
			mDurationDist[duration] = Vector2f(finalBallPos[0], finalBallPos[1]).length();
		}
	}
	data.close();
}


shared_ptr<Action> Shoot::perform()
{
	/*if( !shouldContinueKick() )											//camera change it																//camera add it
	{
		printf("nani???!!!\n");
		mDuration = 0.0f;
		mSubTaskList.clear();
		LOG_PRINT("dura","stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		return FAT.controlPreferThan("squat", "*");
	}*/ //TT test remove

	//printf("mSubTaskList.size()=%d\n", mSubTaskList.size() );
	//printf("Shoot: mStartTime=%f\tmSubTaskList.size()=%d\n",mStartTime,mSubTaskList.size());

	if(mIsFirstTime)
	{
		mIsFirstTime=false;
		if(mStartTime<0)
		{
			//FAT.setCurrentTask(mRaiseRFootTask);//????????????????????????????
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
			if(mIsLeftLeg){
				printf("%s\n","set current: raise L");
				FAT.setCurrentTask(mRaiseLFootTask);
			}
			else{
				printf("%s\n","set current: raise R");
				FAT.setCurrentTask(mRaiseRFootTask);
			}
		}
	}

	/*
	if( mSubTaskList.size() > 0 )
	{
		if(mStartTime<0.0f)
		{
			FAT.setCurrentTask(mRaiseRFootTask);//????????????????????????????
			mStartTime=WM.getSimTime();

			//TT try=====================================
			//mDuration=FAT.calTaskTime(mRaiseRFootTask) + mMoveCoMDuration*2 + mSquatDuration;
			//mDuration=FAT.calTaskTime(mRaiseRFootTask);
			//mDuration=4.0f;
			//==============================================
		}

		act=Task::perform();

		if( NULL != act.get() )
		{
			//printf("%s\n","---return current task---");
			return act;
		}
		else	//start raise foot
		{
			if(mIsLeftLeg){
				//printf("%s\n","set current: raise L");
				FAT.setCurrentTask(mRaiseLFootTask);
			}
			else{
				//printf("%s\n","set current: raise R");
				FAT.setCurrentTask(mRaiseRFootTask);
			}
		}
	}
	*/

	string currentTaskName=FAT.currentTaskName();
	//cout<<"--------"<<currentTaskName<<"---------"<<endl;

	////////////////////////////////////////////////////
	if( "null"==currentTaskName || "squat"==currentTaskName || "*"==currentTaskName ) /////////////// * ???
	{
		//cout<<"xxxxxxxxxxxx "<<currentTaskName<<" xxxxxxxxxxxxxx"<<endl;
		return FAT.controlPreferThan("squat", "*");
	}
	else
	{
		return FAT.controlPreferThan(currentTaskName,"*");
	}
	////////////////////////////////////////////////////

	//TT rewrite
	/*
	if( currentTaskName == mRaiseRFootTask || currentTaskName == mRaiseLFootTask )
	{
		LOG_FLUSH;
		if(mIsLeftLeg)
		{
			LOG_PRINT("dura","raiseL");
			return FAT.controlPreferThan(mRaiseLFootTask, "*");
		}
		else
		{
			LOG_PRINT("dura","raiseR");
			return FAT.controlPreferThan(mRaiseRFootTask, "*");
		}
	}

	else if( currentTaskName == mBaiRtuiTask )
	{
		LOG_PRINT("dura","hu_kick3R");
		return FAT.controlPreferThan("hu_kick3R", "*");
	}

	else if( currentTaskName == mBaiLtuiTask )
	{
		LOG_PRINT("dura","hu_kick3L");
		return FAT.controlPreferThan("hu_kick3L", "*");
	}

	else if( currentTaskName == mAccRTask )
	{
		return FAT.controlPreferThan("hu_kick4R", "*");
		LOG_PRINT("dura","hu_kick4R");
	}

	else if( currentTaskName == mAccLTask )
	{
		LOG_PRINT("dura","hu_kick4L");
		return FAT.controlPreferThan("hu_kick4L", "*");
	}

	else
	{
		LOG_PRINT("dura","squat");
		return FAT.controlPreferThan("squat", "*");
	}
	*/

} //end of Shoot::perform()


} //end of namespace task

