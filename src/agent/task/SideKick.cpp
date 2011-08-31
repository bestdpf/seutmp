/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id $
 *
 ****************************************************************************/
 
#include "SideKick.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"
#include "Walk.h"
#include "MoveCoM.h"
#include "core/WorldModel.h"
#include <fstream>
#include <sstream>
 
namespace task{
	 
	using namespace boost;
	using namespace action;
	using namespace std;
	using namespace robot::humanoid;
	using namespace controller;
	using namespace perception;

    DEFINE_STATIC_GRAPHIC_LOGGER(SideKick)
	 
	SideKick::SideKick( const math::Vector2f& target, bool isLeftLeg,Task* primary ):
		 BasicKick( target, Vector2f(-0.15, 0.12), -90, isLeftLeg, primary)
	{
		static bool isFirstTime = true;
		 
		mRaiseRFootTask = "sidekick1R";
		mRaiseLFootTask = "sidekick1L";
		mBaituiTask = "sidekick2";
		mAccTask = "sidekick3";
		
		if( isFirstTime )
		{
			isFirstTime = false;
			generateAnkleDistTable();
			generateDurationDistTable ();
		}

BEGIN_ADD_STATIC_LOG_LAYER(SideKick)
	ADD_LOG_LAYER("desiredDuration");
	ADD_LOG_LAYER("KickParameter");
	ADD_LOG_LAYER("amendJointPose");
	ADD_LOG_LAYER("amendBaituiJointPose");
END_ADD_STATIC_LOG_LAYER(SideKick);
	}
	 
	AngDeg SideKick::getDesiredAnkleAngle()
	{
		const Vector2f posBall2D = WM.getBallGlobalPos2D();
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
	 
	float SideKick::getDesiredDuration ()
	{
		 //
		return 0.1;
		 
		const Vector2f posBall2D = WM.getBallGlobalPos2D();
		float dist = (posBall2D-mTarget).length();
		float err = 999;
		float duration = 0;
		for( TDurationDist::const_iterator iter = mDurationDist.begin(); iter !=mDurationDist.end(); iter ++ )
		{
			LOG_PRINTF("desiredDuration", "%f", abs(iter->second - dist));
			if( abs(iter->second - dist) < err )	// The current duration can make the ball closer to the target.
			{
				err = abs(iter->second - dist);
				duration = iter->first;
			}
		}
		 
		return duration;
	}
	 
	void SideKick::generateAnkleDistTable()
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
	 
	void SideKick::generateDurationDistTable()
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
	 
}
