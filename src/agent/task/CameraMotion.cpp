/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: CameraMotion.cpp
 *
 ****************************************************************************/

#include <boost/shared_ptr.hpp>

#include "CameraMotion.h"
#include "perception/Vision.h"
#include "core/WorldModel.h"
#include "controller/Timing.h"
#include "configuration/Configuration.h"
#include "robot/humanoid/Nao.h"
#include "action/Actions.h"

#define TURN_SPEED_FOR_SEARCH 350 //max speed is about 351

namespace task {

using namespace boost;
using namespace action;
using namespace perception;
using namespace std;
using namespace controller;
using namespace robot::humanoid;
using namespace serversetting;

DEFINE_STATIC_GRAPHIC_LOGGER(CameraMotion);


CameraMotion::CameraMotion(int mode, Task* primary) : Task(-1, primary)
{
	mMode=mode;
	/*BEGIN_ADD_STATIC_LOG_LAYER(CameraMotion)
		ADD_LOG_LAYER("TT")
	END_ADD_STATIC_LOG_LAYER(CameraMotion)*/
}

CameraMotion::~CameraMotion()
{
}


shared_ptr<Action> CameraMotion::perform()
{
	if(0==mMode)
		return dontTurn();
	else if(1==mMode)
		return searchBall();
	else if(2==mMode)
		return stickToBall();
	else if(3==mMode)
		return searchFlags();
	else if(4==mMode)
		return forAttacking();
	else if(-2==mMode)
		return forTest();


	static int countAfterGreat=0;

	const bool seeBall=WM.canSeeBall();
	const int flags=WM.seenFlagsNum();
	float chaosTime=WM.timeAfterLastSeeEnoughFlags();

	if( seeBall && flags>=2 )
	{
		//Great!
		countAfterGreat=0;
		return dontTurn();
	}

	else if( false==seeBall && countAfterGreat<10 )
	{
		//last time Great
		countAfterGreat++;
		return searchBallAfterGreat();
	}

	else if( false==seeBall && flags>=2 )
	{
		// can see only flags, search ball
		countAfterGreat=100;
		return searchBall();
	}

	else if( true==seeBall && flags<=1 )
	{
		countAfterGreat=100;
		if(chaosTime<2)
		{
			//cout<<"inteTime is "<<inteTime<<"   , stickToBall";
			return stickToBall();
		}

		/*if(distToBall<1.3){
			// can see ball and distToBall<1.3
			return stickToBall();
		}*/

		else{
			// search falgs
			return searchFlags();
		}
	}

	else
	{
		// Who turned off the light?
		countAfterGreat=100;
		return searchBall();
	}
}


shared_ptr<JointAction> CameraMotion::searchBallAfterGreat()
{
	shared_ptr<JointAction> jact(new JointAction(false));
	const Vector3f& ballLastPol=WM.getBallLaPol();
	const float angX=ballLastPol.y();
	const float angY=ballLastPol.z();

	if(angX<0)
		jact->setForCamera(0,-TURN_SPEED_FOR_SEARCH);		//last see ball at right, so search right
	else
		jact->setForCamera(0,TURN_SPEED_FOR_SEARCH);

	if(angY<0)
		jact->setForCamera(1,-TURN_SPEED_FOR_SEARCH);		//last see ball downward, so search bottom
	else
		jact->setForCamera(1,TURN_SPEED_FOR_SEARCH);

	return jact;
}


shared_ptr<JointAction> CameraMotion::searchBall()						//shaking head!!!
{
	shared_ptr<JointAction> jact(new JointAction(false));
	const JointPerception& lastJP= WM.lastPerception().joints();
	float ang0= lastJP[0].angle();
	float ang1= lastJP[1].angle();

	float lastVX=WM.getSearchSpeed().x();

	//TT: set(jid,v) v is angle-degree per second
	//TT: max speed is about 351 deg/s
	//TT: I want ang1 to change 1 angle-degree everytime it comes into this function (about 0.06s).
	//    1/0.06=16, so I set 16 on Y direction.

	//X
	if( ang0>=115 )
		jact->setForCamera(0,-TURN_SPEED_FOR_SEARCH); //turn right
	else if( ang0<=-115 )
		jact->setForCamera(0,TURN_SPEED_FOR_SEARCH); //turn left
	else
	{
		if(lastVX>=0)
			jact->setForCamera(0,TURN_SPEED_FOR_SEARCH);
		else
			jact->setForCamera(0,-TURN_SPEED_FOR_SEARCH);
	}

	//Y
	if(ang1>-30)
		jact->setForCamera(1,-100);
	else if(ang1<-40)
		jact->setForCamera(1,100);
	else
		jact->setForCamera(1,0);

	return jact;
}


shared_ptr<JointAction> CameraMotion::stickToBall()
{
	shared_ptr<JointAction> jact(new JointAction(false));
	const Vector3f& ballLastPol=WM.getBallLaPol();
	float angX=ballLastPol.y();
	float angY=ballLastPol.z();

	float vx=200;
	float vy=200;

	if(angX>10)
		jact->setForCamera(0,vx);
	else if(angX<-10)
		jact->setForCamera(0,-vx);
	else
		jact->setForCamera(0,0);

	if(angY>-30)
		jact->setForCamera(1,vy);
	else if(angY<-50)
		jact->setForCamera(1,-vy);
	else
		jact->setForCamera(1,0);

	return jact;
}


shared_ptr<JointAction> CameraMotion::searchFlags()
{
	shared_ptr<JointAction> jact(new JointAction(false));
	const JointPerception& lastJP= WM.lastPerception().joints();
	float ang0= lastJP[0].angle();
	float ang1= lastJP[1].angle();

	float lastVX=WM.getSearchSpeed().x();

	//TT: set(jid,v) v is angle-degree per second
	//TT: max speed is about 351 deg/s
	//TT: I want ang1 to change 1 angle-degree everytime it comes into this function (about 0.06s).
	//    1/0.06=16, so I set 16 on Y direction.

	//X
	if( ang0>=115 )
		jact->setForCamera(0,-TURN_SPEED_FOR_SEARCH); //turn right
	else if( ang0<=-115 )
		jact->setForCamera(0,TURN_SPEED_FOR_SEARCH); //turn left
	else
	{
		if(lastVX>=0)
			jact->setForCamera(0,TURN_SPEED_FOR_SEARCH);
		else
			jact->setForCamera(0,-TURN_SPEED_FOR_SEARCH);
	}

	//Y
	if(ang1>5)
		jact->setForCamera(1,-100);
	else if(ang1<-5)
		jact->setForCamera(1,100);
	else
		jact->setForCamera(1,0);

	return jact;
}


shared_ptr<JointAction> CameraMotion::dontTurn()
{
	shared_ptr<JointAction> jact(new JointAction(false));
	jact->setForCamera(0,0);
	jact->setForCamera(1,0);
	return jact;
}


shared_ptr<JointAction> CameraMotion::forAttacking()
{
	static int countAfterGreat=0;
	static int countCantSeeGoal=0;

	bool seeBall=WM.canSeeBall();
	bool seeGoal= ( WM.canSeeFlag(Vision::G1R) || WM.canSeeFlag(Vision::G2R) );

	if( seeBall && seeGoal )
	{
		//Great!
		countAfterGreat=0;
		countCantSeeGoal=0;
		return dontTurn();
	}

	else if( !seeBall && countAfterGreat<10 )
	{
		//just now Great
		countAfterGreat++;
		if(seeGoal)
			countCantSeeGoal=0;
		else
			countCantSeeGoal++;
		return searchBallAfterGreat();
	}

	else if( !seeBall && seeGoal )
	{
		//can see only flags, search ball
		//countAfterGreat=256;
		countCantSeeGoal=0;
		return searchBall();
	}

	else if( seeBall && !seeGoal )
	{
		//countAfterGreat=256;
		countCantSeeGoal++;

		if(countCantSeeGoal<50){
			return stickToBall();
		}

		else{
			//search falgs
			return searchFlags();
		}
	}

	else
	{
		//Who turned off the light?
		//countAfterGreat=256;
		countCantSeeGoal++;
		//return searchBall();
		return searchFlags();
	}
}


shared_ptr<JointAction> CameraMotion::forTest()
{
	return searchFlags();
	shared_ptr<JointAction> jact(new JointAction(false));
	jact->setForCamera(0,10000);
	jact->setForCamera(1,-10000);
	return jact;
}



} //end of namespace task

