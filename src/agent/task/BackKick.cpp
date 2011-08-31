/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 * $Id: BackKick.cpp         Tue Jul  8 21:25:42 2008    $ 
 ****************************************************************************/
 
 
#include "BackKick.h"
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
	
	
	DEFINE_STATIC_GRAPHIC_LOGGER(BackKick)
	
	
	BackKick::BackKick(const math::Vector2f& target,bool isLeftLeg,Task* primary):
	            BasicKick(target, Vector2f(-0.16, -0.05), 180.0, isLeftLeg, primary)
	{
		static bool isFirstTime = true;
		
		 mRaiseRFootTask = "hu_back_2R";
		 mRaiseLFootTask = "hu_back_2L";
		 mBaituiTask = "hu_back_3";											/////terrymimi
		 mAccTask = "hu_back_4";												/////terrymimi
		 
		 if( isFirstTime )
		 {
			 isFirstTime = false;
			 generateAnkleDistTable();
			 generateDurationDistTable ();
		 }
		 
BEGIN_ADD_STATIC_LOG_LAYER(BackKick)
	ADD_LOG_LAYER("desiredDuration");
	ADD_LOG_LAYER("KickParameter");
	ADD_LOG_LAYER("amendJointPose");
	ADD_LOG_LAYER("amendBaituiJointPose");
END_ADD_STATIC_LOG_LAYER(BackKick);
		}
	AngDeg BackKick::getDesiredAnkleAngle()
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
	 
	 float BackKick::getDesiredDuration ()
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
	 
	 void BackKick::generateAnkleDistTable()
	 {
		 ifstream data;
		 data.open("data/BackKick_rankle-dist.data");
		 
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
	 
	 void BackKick::generateDurationDistTable()
	 {
		 ifstream data;
		 data.open("data/BackKick_duration-dist.data");
		 
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
