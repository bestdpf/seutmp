
/***************************************************************************
*                              SEU RoboCup Simulation Team
*                     -------------------------------------------------
* Copyright (c) Southeast University , Nanjing, China.
* All rights reserved.
*
* $Id$
*
****************************************************************************/

#include "KickTask.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"
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

DEFINE_STATIC_GRAPHIC_LOGGER(KickTask)


KickTask::KickTask( const string& firstTaskName, Task* primary ):
	BasicKick( Vector2f(0,0), Vector2f(0,0), 2.0f, false, primary )
{
	mIsFirstTime=true;
	static bool isFirstTime = true;
	mRaiseRFootTask=firstTaskName;
	mDuration=FAT.calTaskTime(mRaiseRFootTask) + mSquatDuration;
	//mDuration=FAT.calTaskTime(mRaiseRFootTask) + mMoveCoMDuration*2 + mSquatDuration; //calculated in BasicKick

	if(isFirstTime)
	{
		isFirstTime=false;
		//generateAnkleDistTable();
		//generateDurationDistTable();
	}

	/*BEGIN_ADD_STATIC_LOG_LAYER(KickTask)
	ADD_LOG_LAYER("supportfoot");
	ADD_LOG_LAYER("DesiredTorso");
	ADD_LOG_LAYER("terrymimi");
	ADD_LOG_LAYER("dura");
	END_ADD_STATIC_LOG_LAYER(KickTask);*/
}


AngDeg KickTask::getDesiredAnkleAngle()
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


float KickTask::getDesiredDuration()
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


void KickTask::generateAnkleDistTable()
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


void KickTask::generateDurationDistTable()
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


shared_ptr<Action> KickTask::perform()
{
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
				//printf("%s\n","set current: raise L");
				FAT.setCurrentTask(mRaiseLFootTask);
			}
			else{
				//printf("%s\n","set current: raise R");
				FAT.setCurrentTask(mRaiseRFootTask);
			}
		}
	}


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

} //end of KickTask::perform()


} //end of namespace task

