 /***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Player.cpp 2755 2009-04-02 08:00:08Z zyj $
 *
 ****************************************************************************/

#include "Player.h"
#include "core/WorldModel.h"
#include "action/InitAction.h"
#include "action/BeamAction.h"
#include "action/Say.h"
#include "action/Actions.h"
#include "configuration/Configuration.h"
#include "controller/FixedAngleTrace.h"
#include "task/Walk.h"
#include "task/WalkRel.h"
#include "task/Kick.h"
#include "math/TLine2.hpp"
#include "math/Math.hpp"
#include "math/TConvexPolygon.hpp"
#include "task/MoveCoM.h"
#include "task/Shoot.h"
#include "task/KickTask.h"
#include "task/WalkSlow.h"
#include "task/CameraMotion.h"

#include "core/SayAndHearModel.h"
#include<fstream>//////////////////////////////////////////////////////TT test

#include<time.h>

namespace soccer {

using namespace std;
using namespace boost;
using namespace serversetting;
using namespace controller;
using namespace perception;
using namespace action;
using namespace task;


Player::Player()
    :mTask(-1,NULL)
{
	//load data of KickMotion
	mKickMotionVector.clear();
	ifstream inFile("data/kick_motion.txt",ios::in);
	if(NULL!=inFile)
	{
		string oneLine;
		KickMotion tempKM;
		while( !inFile.eof() )
		{
			getline(inFile,oneLine); //maybe tempKM.firstTaskName

			if( oneLine.empty() ) continue; //skip empty line
			if( '#'==oneLine[0] || ' '==oneLine[0] ) continue; //skip comment line

			tempKM.firstTaskName=oneLine;
			inFile>>tempKM.kickTargetRel.x()>>tempKM.kickTargetRel.y()
				  >>tempKM.myDesiredRelPosToBall.x()>>tempKM.myDesiredRelPosToBall.y()
				  >>tempKM.relPosToBallToStopWalk.x()>>tempKM.relPosToBallToStopWalk.y();

			mKickMotionVector.push_back(tempKM);
		}

		inFile.close();
	}
}

Player::~Player()
{
}

bool Player::init()
{
    if ( !Agent::init() ) return false;

    // get the respond (first message) from the server
    shared_ptr<Perception> p = sense();
    if ( 0 == p.get() ) {
        return false;
    }
    if ( !WM.update(p) ) return false;

    // scend the init message
    shared_ptr<Action> iAct(new InitAction(OPTS.arg<string>("teamname"),
										   OPTS.arg<unsigned int>("unum")));
    perform( iAct );

	//Allen, for GUI under new server
	sense();
    shared_ptr<Action> bAct = shared_ptr<Action > (new BeamAction(FM.getMy().beforeKickOffBeam));
    perform(bAct);
	//===============================

    return true;
}

/**
 * @author allen
 */
boost::shared_ptr<Action> Player::mysay(int p_recvNum,bool p_isFall,
										float p_bx,float p_by,
										float p_rx,float p_ry)
{
	Message msg;
    char * test;
    mVector3f ballPos;
    mVector3f roboPos;
    ballPos.x = p_bx;
    ballPos.y = p_by;
    roboPos.x = p_rx;
    roboPos.y = p_ry;

	test = msg.makeSen1(test,9,true,ballPos,roboPos);
    string test1 = test;

	shared_ptr<Say> say(new Say(test1));
	return say;
}

void Player::myhear()
{
	const vector<shared_ptr<perception::Hear> >& hear = WM.lastPerception().hear();
	FOR_EACH( iter, hear )
	{
		const string& msg = (*iter)->message();

		Message msg1;
		if(msg1.DestructDirectMsg(msg.c_str()))
		{
		}
		else
		{
		}

		WM.setNowFormation(msg1.getFormation());
		WM.setBestZone(msg1.getZoneX(),msg1.getZoneY());
		WM.setHearOurFastestToBallNum(msg1.getFastestNum());
	}
}

/**
 * the entry of "Think" thread
 * 1-GK shout
 * 2-different play mode
 * 3-CameraMotion
 *
 * @author Xu Yuan
 *
 * @return boost::shared_ptr<Action>
 */
boost::shared_ptr<Action> Player::think()
{
	boost::shared_ptr<Actions> actions(new Actions());
     //   SHM.update();
        if(SHM.IsCanSay()){
            	shared_ptr<Say> SHMsay(new Say(SHM.getSayString()));
		actions->add(SHMsay);
        }
       // SHM.printhear();
      //  SHM.printsay();

//	//GK shout
//	if( configuration::Formation::PT_GOALKEEPER==FM.getMy().type )
//	{
//		WM.setNowFormation(chooseFormation());
//		Message saymsg;
//		PassZoneStrategy pzs;
//		WM.setBestZone(pzs.getBestX(),pzs.getBestY());
//		int saynum = 0;
//		if(WM.getOurPlayerNumber()>=3){
//			WM.setHearOurFastestToBallNum(pzs.getOurFastestPlayer());
//			saynum=pzs.getOurFastestPlayer();
//		}else{
//			WM.setHearOurFastestToBallNum(0);
//			saynum=0;
//		}
//		shared_ptr<Say> FTsay(new Say(saymsg.MakeDirectMsg(WM.getNowFormation(),WM.getBestZoneX(),WM.getBestZoneY(),saynum)));
//		actions->add(FTsay);
//	}
//	WM.setHearOurFastestToBallNum(0);
//	myhear();

	//TT add for controling CameraMotion
	mCameraMotionMode=-1; //mCameraMotionMode will be changed in "play mode"
//cout << "fast"<<WM.getOurFastestToBallNum()<<endl;
// cout << "mCanPass" << PM.canPass() << "coohse"<<PM.choosePass()<< "bestid" << PM.getBestPassID()<<endl;
	//play mode
	switch ( WM.getPlayMode() )
    {
        case PM_BEFORE_KICK_OFF:
			actions->add(playBeforeKickOff());
            break;
        case PM_KICK_OFF_LEFT:
        case PM_KICK_OFF_RIGHT:
			actions->add(playKickOff());
            break;
        case PM_PLAY_ON:
			actions->add(playPlayOn());
            break;
        case PM_KICK_IN_LEFT:
        case PM_KICK_IN_RIGHT:
			actions->add(playKickIn());
            break;
        case PM_CORNER_KICK_LEFT:
        case PM_CORNER_KICK_RIGHT:
			actions->add(playCornerKick());
            break;
        case PM_GOAL_KICK_LEFT:
        case PM_GOAL_KICK_RIGHT:
            actions->add(playGoalKick());
			break;
        case PM_OFFSIDE_LEFT:
        case PM_OFFSIDE_RIGHT:
            actions->add(playOffSide());
			break;
        case PM_GAME_OVER:
			actions->add(playGameOver());
            break;
        case PM_GOAL_LEFT:
        case PM_GOAL_RIGHT:
            actions->add(playGoal());
			break;
        case PM_FREE_KICK_LEFT:
        case PM_FREE_KICK_RIGHT:
            actions->add(playFreeKick());
			break;
        default:
            cerr<<"[WARNING] Player can not handle this Play Mode!\n";
            actions->add(playPlayOn());
			break;
    }

	//camera motion
	shared_ptr<JointAction> jact(new JointAction(false));
	if(NULL==WM.lastPerception().vision().get())
	{
		jact->setForCamera(0,WM.getSearchSpeed().x());
		jact->setForCamera(1,WM.getSearchSpeed().y());
		actions->add(jact);
	}
	else
	{
		shared_ptr<CameraMotion> cm( new CameraMotion(mCameraMotionMode) );
		actions->add(cm->perform());
	}
	return actions;
}

shared_ptr<Action> Player::playKickOff()
{
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ( ( TI_LEFT == ti && PM_KICK_OFF_LEFT == pm )
         || ( TI_RIGHT == ti && PM_KICK_OFF_RIGHT == pm ) )
    {
        return playOurKickOff();
    } else {
        return playOppKickOff();
    }
}

shared_ptr<Action> Player::playKickIn()
{
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ( ( TI_LEFT == ti && PM_KICK_IN_LEFT == pm )
         || ( TI_RIGHT == ti && PM_KICK_IN_RIGHT == pm ) )
    {
        return playOurKickIn();
    } else {
        return playOppKickIn();
    }
}

shared_ptr<Action> Player::playCornerKick()
{
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ( ( TI_LEFT == ti && PM_CORNER_KICK_LEFT == pm )
         || ( TI_RIGHT == ti && PM_CORNER_KICK_RIGHT == pm ) )
    {
        return playOurCornerKick();
    } else {
        return playOppCornerKick();
    }
}

shared_ptr<Action> Player::playGoalKick()
{
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ( ( TI_LEFT == ti && PM_GOAL_KICK_LEFT == pm )
         || ( TI_RIGHT == ti && PM_GOAL_KICK_RIGHT == pm ) )
    {
        return playOurGoalKick();
    } else {
        return playOppGoalKick();
    }
}

shared_ptr<Action> Player::playOffSide()
{
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ( ( TI_LEFT == ti && PM_OFFSIDE_LEFT == pm )
         || ( TI_RIGHT == ti && PM_OFFSIDE_RIGHT == pm ) )
    {
        return playOurOffSide();
    } else {
        return playOppOffSide();
    }
}

shared_ptr<Action> Player::playGoal()
{
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ( ( TI_LEFT == ti && PM_GOAL_LEFT == pm )
         || ( TI_RIGHT == ti && PM_GOAL_RIGHT == pm ) )
    {
        return playOurGoal();
    } else {
        return playOppGoal();
    }
}

shared_ptr<Action> Player::playFreeKick()
{
    TTeamIndex ti = WM.getOurTeamIndex();
    TPlayMode pm = WM.getPlayMode();
    if ( ( TI_LEFT == ti && PM_FREE_KICK_LEFT == pm )
         || ( TI_RIGHT == ti && PM_FREE_KICK_RIGHT == pm ) )
    {
        return playOurFreeKick();
    } else {
        return playOppFreeKick();
    }
}
/***********************************Gravity test********************************/
Direction Player::calFallDirPenalty(){
    Vector2f ballPos=WM.getBallGlobalPos2D();
    float xBall = ballPos[0];
    float yBall = ballPos[1];
    Vector2f myPos = WM.getMyGlobalPos2D();
    float xMe = myPos[0];
    float yMe = myPos[1];
     /*if(yBall>0){
    if(yMe<yBall)return LEFT;
    else if(yMe>yBall) return RIGHT;
        else return UNKNOWN;
    }
    else if(yBall<0){
        if(yMe>yBall)return RIGHT;
    else if(yMe<yBall) return LEFT;
        else return UNKNOWN;
    }
    else return UNKNOWN;
      */
    Vector3f ballVel = WM.getBallGlobalVel();
    Vector2f ballVel2D(ballVel.x(), ballVel.y());
    float ballVelx=ballVel2D[0];
    float ballVely=ballVel2D[1];
    Vector2f ballRelVel=WM.getBallRelVel2D();
    float ballRelVelx=ballRelVel[0];
    float ballRelVely=ballRelVel[1];
    if((ballVely/ballVelx*(xMe-xBall)+yBall)>yMe){
        //cout<<"ballVely="<<ballVely<<endl;
        //cout<<"左倒"<<endl;
        //printf("%.2f\n",ballVely);
        return LEFT;
    }
    else { if((ballVely/ballVelx*(xMe-xBall)+yBall)>yMe)return RIGHT;
            else return UNKNOWN;
          }
}

bool Player::whetherToFall(){
    Vector2f ballPos=WM.getBallGlobalPos2D();
    float xBall = ballPos[0];
    float yBall = ballPos[1];
    Vector3f ballVel = WM.getBallGlobalVel();
    Vector2f ballVel2D(ballVel.x(), ballVel.y());
    float ballVelx=ballVel2D[0];
    float ballVely=ballVel2D[1];
    Vector2f oppgoal(-half_field_length, 0);
    Vector2f penaltyright(penalty_length - half_field_length, -half_penalty_width);
    Vector2f penaltyleft(penalty_length - half_field_length, half_penalty_width);
    AngDeg ang = (ballPos - oppgoal).angle();
    AngDeg angLBoder = (penaltyleft - oppgoal).angle();
    AngDeg angRBoder = (penaltyright - oppgoal).angle();
    Vector2f ballRelVel=WM.getBallRelVel2D();
    float ballRelVelx=ballRelVel[0];
    float ballRelVely=ballRelVel[1];
    if (isAngInInterval(ang,angRBoder,angLBoder)){
    if(xBall<-5.8&&abs(ballVelx)>=4.0)return true;
    else return false;
    }
    else{if (abs(yBall)<5.8&&abs(ballVely)>=4.0)return true;
         else return false;
        }
}
/**********************************************Gravity test******************************/

/******************************Gravity test*************************************/
boost::shared_ptr<action::Action> Player::penaltyKiller()
{

    Direction fallDir;
    bool isit;
    bool isIn;
    bool isBallWillIn = isBallWillInAfterNSteps(65);
    GoalKeeperState state=updateGoalKeeperState();
    shared_ptr<Action> act;
    shared_ptr<Task> cTask = mTask.getFirstSubTask();
    shared_ptr<KeepFall> cKeepFall = shared_dynamic_cast<KeepFall>(cTask);
    shared_ptr<Fall> cFall = shared_dynamic_cast<Fall>(cTask);
    shared_ptr<Walk> cFastWalk= shared_dynamic_cast<Walk>(cTask);
    shared_ptr<Walk> cWalk= shared_dynamic_cast<Walk>(cTask);
    act=mTask.perform();

	Vector3f ballVel=WM.getBallGlobalVel();
	Vector3f ballPos=WM.getBallGlobalPos();
	act.reset();
        shared_ptr<Task> leftFall=shared_ptr<Task> (new Fall(&mTask,LEFT));
        shared_ptr<Task> rightFall=shared_ptr<Task> (new Fall(&mTask,RIGHT));
        shared_ptr<Task> frontFall=shared_ptr<Task> (new Fall(&mTask,FRONT));
        shared_ptr<Task> backFall=shared_ptr<Task> (new Fall(&mTask,BACK));
	isIn = isBallWillIn && WM.isBallToOurGoal();
	float bodyHeight = WM.getMyGlobalPos().z();
	Vector3f ballPosToBone = WM.getPosToBone(ballPos);
	float currentYDist = abs( ballPosToBone.y() );


                       fallDir=calFallDirPenalty();
                       isit=whetherToFall();
		/*	//if(isBallWillCrossMeAfterNStps(65)&&isIn&&cFall.get()==NULL
						//&&bodyHeight<0.2f&&FAT.isPoseReached("pituipq",7.0f))

                       if(isit)
			{
				if(NULL != cKeepFall.get()){
					return mTask.perform();
				}
				else{
					mTask.clear();
					mTask.append(shared_ptr<Task> (new KeepFall(&mTask,LEFT)));
					act = mTask.perform();
					return act;
				}
			}
			if(UNKNOWN == fallDir)
				return mBalance.perform();
                        if( (fallDir==BACK || fallDir==RIGHT) && currentYDist > 0.3f)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
                                        mTask.append(backFall);//fall to right
					act=mTask.perform();
				}
			}
                        else if( (fallDir==FRONT || fallDir==LEFT) && currentYDist > 0.3f)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
                                        mTask.append(frontFall);//fall to left
					act=mTask.perform();
				}
			}
			else
				act = mBalance.perform();
			//break;
		//------------------------------------------up: lying/diving and balance
*/
  switch (state)
	{
		case RIGHTFALL_STATE:
		case LEFTFALL_STATE:
		case DIVED_STATE:
			if(isit)
			{
				if(cKeepFall.get()!=NULL){
					act=mTask.perform();
				}
				else{
					mTask.clear();
					mTask.append(shared_ptr<Task> (new KeepFall(&mTask,RIGHT)));
					act= mTask.perform();
				}
			}
			else
			{
				mTask.clear();
				act=mBalance.perform();
			}
			break;
		//--------------------------------------------up: dived or falled

		case LIED_STATE:
			act=mBalance.perform();
			break;
		//--------------------------------------------up: lied

		case LYING_STATE:
		case DIVING_STATE:
		case BALANCE_STATE:
                        fallDir=calFallDirPenalty();
                        isit=whetherToFall();
		        //fallDir=calFallDirPenalty();//decide whether agent should Fall,and predict the direction
			if(isit)//&&isIn&&cFall.get()==NULL
						//&&bodyHeight<0.2f&&FAT.isPoseReached("pituipq",7.0f))
			{
				if(NULL != cKeepFall.get()){
					return mTask.perform();
				}
				else{
					mTask.clear();
					mTask.append(shared_ptr<Task> (new KeepFall(&mTask,LEFT)));
					act = mTask.perform();
					return act;
				}
			}
			if(UNKNOWN == fallDir)
				return mBalance.perform();
			if(fallDir==RIGHT && currentYDist > 0.3f)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
					mTask.append(rightFall);//fall to right
					act=mTask.perform();
				}
			}
			else if(fallDir==LEFT && currentYDist > 0.3f)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
					mTask.append(leftFall);//fall to left
					act=mTask.perform();
				}
			}
			else
				act = mBalance.perform();
			break;
		//------------------------------------------up: lying/diving and balance

		default:
			act=mBalance.perform();
			break;
	}
	return act;
}
/******************************Gravity test*************************************/
/////////////////////////////////////////////////////////////////////////
///////////////////////////// Skills ////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

boost::shared_ptr<action::Action> Player::keepGoal()
{
	Direction fallDir;
	bool isIn;
	bool isBallWillIn = isBallWillInAfterNSteps(65);
	GoalKeeperState state=updateGoalKeeperState();//get the state of goalKeeper
	shared_ptr<Action> act;
	shared_ptr<Task> cTask = mTask.getFirstSubTask();
	shared_ptr<KeepFall> cKeepFall = shared_dynamic_cast<KeepFall>(cTask);
	shared_ptr<Fall> cFall = shared_dynamic_cast<Fall>(cTask);
	shared_ptr<Walk> cFastWalk= shared_dynamic_cast<Walk>(cTask);
	shared_ptr<Walk> cWalk= shared_dynamic_cast<Walk>(cTask);
	act=mTask.perform();

	if(cWalk.get() == NULL && cFastWalk.get()==NULL&&cKeepFall.get()==NULL&&NULL!=act.get())
	{
		return act; //if current task is not KeepFall and not empty ,just do it
	}

	Vector3f ballVel=WM.getBallGlobalVel();
	Vector3f ballPos=WM.getBallGlobalPos();
	act.reset();
	shared_ptr<Task> leftFall=shared_ptr<Task> (new Fall(&mTask,LEFT));
	shared_ptr<Task> rightFall=shared_ptr<Task> (new Fall(&mTask,RIGHT));
	isIn = isBallWillIn && WM.isBallToOurGoal();
	float bodyHeight = WM.getMyGlobalPos().z();
	Vector3f ballPosToBone = WM.getPosToBone(ballPos);
	float currentYDist = abs( ballPosToBone.y() );

	switch (state)
	{
		case RIGHTFALL_STATE:
		case LEFTFALL_STATE:
		case DIVED_STATE:
			if(isIn)
			{
				if(cKeepFall.get()!=NULL){
					act=mTask.perform();
				}
				else{
					mTask.clear();
					mTask.append(shared_ptr<Task> (new KeepFall(&mTask,RIGHT)));
					act= mTask.perform();
				}
			}
			else
			{
				mTask.clear();
				act=mBalance.perform();

			}
			break;
		//--------------------------------------------up: dived or falled

		case LIED_STATE:
			act=mBalance.perform();
			break;
		//--------------------------------------------up: lied

		case LYING_STATE:
		case DIVING_STATE:
		case BALANCE_STATE:
			fallDir=calFallDirection(65);//decide whether agent should Fall,and predict the direction
			if(isBallWillCrossMeAfterNStps(65)&&isIn&&cFall.get()==NULL
						&&bodyHeight<0.2f&&FAT.isPoseReached("pituipq",7.0f))
			{
				if(NULL != cKeepFall.get()){
					return mTask.perform();
				}
				else{
					mTask.clear();
					mTask.append(shared_ptr<Task> (new KeepFall(&mTask,LEFT)));
					act = mTask.perform();
					return act;
				}
			}
			if(UNKNOWN == fallDir)
				return mBalance.perform();
			if(fallDir==RIGHT && currentYDist > 0.3f)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
					mTask.append(rightFall);//fall to right
					act=mTask.perform();
				}
			}
			else if(fallDir==LEFT && currentYDist > 0.3f)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
					mTask.append(leftFall);//fall to left
					act=mTask.perform();
				}
			}
			else
				act = mBalance.perform();
			break;
		//------------------------------------------up: lying/diving and balance

		default:
			act=mBalance.perform();
			break;
	}
	return act;
}

boost::shared_ptr<action::Action> Player::keepGoal_TT(int d)
{
	bool isIn;
	bool isBallWillIn = isBallWillInAfterNSteps(65);
	GoalKeeperState state=updateGoalKeeperState();//get the state of goalKeeper
	shared_ptr<Action> act;
	shared_ptr<Task> cTask = mTask.getFirstSubTask();
	shared_ptr<KeepFall> cKeepFall = shared_dynamic_cast<KeepFall>(cTask);
	shared_ptr<Fall> cFall = shared_dynamic_cast<Fall>(cTask);
	shared_ptr<Walk> cFastWalk= shared_dynamic_cast<Walk>(cTask);
	shared_ptr<Walk> cWalk= shared_dynamic_cast<Walk>(cTask);
	act=mTask.perform();

	if(cWalk.get() == NULL && cFastWalk.get()==NULL&&cKeepFall.get()==NULL&&NULL!=act.get())
	{
		return act; //if current task is not KeepFall and not empty ,just do it
	}

	Vector3f ballVel=WM.getBallGlobalVel();
	Vector3f ballPos=WM.getBallGlobalPos();
	act.reset();
	shared_ptr<Task> leftFall=shared_ptr<Task> (new Fall(&mTask,LEFT));
	shared_ptr<Task> rightFall=shared_ptr<Task> (new Fall(&mTask,RIGHT));
	isIn = isBallWillIn && WM.isBallToOurGoal();
	float bodyHeight = WM.getMyGlobalPos().z();
	Vector3f ballPosToBone = WM.getPosToBone(ballPos);
	float currentYDist = abs( ballPosToBone.y() );

	switch (state)
	{
		case RIGHTFALL_STATE:
		case LEFTFALL_STATE:
		case DIVED_STATE:
			if(isIn)
			{
				if(cKeepFall.get()!=NULL){
					act=mTask.perform();
				}
				else{
					mTask.clear();
					mTask.append(shared_ptr<Task> (new KeepFall(&mTask,RIGHT)));
					act= mTask.perform();
				}

			}
			else
			{
				mTask.clear();
				act=mBalance.perform();

			}
			break;
		//--------------------------------------------up: dived or falled

		case LIED_STATE:
			act=mBalance.perform();
			break;
		//--------------------------------------------up: lied

		case LYING_STATE:
		case DIVING_STATE:
		case BALANCE_STATE:
			if( isBallWillCrossMeAfterNStps(65)
				&& isIn
				&& cFall.get()==NULL
				&& bodyHeight<0.2f
				&& FAT.isPoseReached("pituipq",7.0f) )
			{
				if( NULL!=cKeepFall.get() ){
					return mTask.perform();
				}
				else{
					mTask.clear();
					mTask.append(shared_ptr<Task> (new KeepFall(&mTask,LEFT)));
					act = mTask.perform();
					return act;
				}
			} //keep fall

			if(2==d)
				return mBalance.perform();

			if(0==d)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
					mTask.append(rightFall);
					act=mTask.perform();
					return act;
				}
			}
			else if(1==d)
			{
				if(cFall.get()==NULL&&bodyHeight>0.25f)
				{
					mTask.clear();
					mTask.append(leftFall);
					act=mTask.perform();
					return act;
				}
			}
			else
				return mBalance.perform();
			break;
		//------------------------------------------up: lying/diving and balance

		default:
			act=mBalance.perform();
			break;
	}
	return act;
}

bool Player::isBallWillInAfterNSteps(int n)
{
	static float simTime;//this variable can make sure that we predict once every sim_step
	static bool isBallWillIn[5]={0};
	Vector3f ballPPos = WM.getBallGlobalPos();
	Vector3f ballPVel = WM.getBallGlobalVel();
	if(abs(simTime - WM.getSimTime()) > 0.001f)
	{
		for(int i=4;i>0;i--)
			isBallWillIn[i]=isBallWillIn[i-1];//record judements before
		for(int i=0; i<n; i++)
			WM.predictBall(ballPPos, ballPVel);

		if(ballPPos.x() <= -half_field_length)
			isBallWillIn[0] = true;
		else
			isBallWillIn[0] = false;

		simTime = WM.getSimTime();
	}

	int temp = 0;
	for(int i=0; i<5; i++)
	{
		if(true == isBallWillIn[i])
			temp += 1;
	}

	if(temp >= 3)
		return true;
	else
		return false;
}

/**
 * predict whether the goalkeeper should fall
 * and judge the falling direction
 */
Direction Player::calFallDirection(int n)
{
	if (!isBallWillCrossMeAfterNStps(n))
	{
		return UNKNOWN;
	}
	if (!WM.isBallToOurGoal())
	{
		return UNKNOWN;
	}
	Vector3f ballPPos = WM.getBallGlobalPos();
	Vector3f ballPVel = WM.getBallGlobalVel();
	Vector3f ballCPos = WM.getBallGlobalPos();
	Vector2f ballVel(ballPVel.x(), ballPVel.y());
	for (int i=0; i<n; i++)
		WM.predictBall(ballPPos, ballPVel);
	Vector3f currentBallPosToBone = WM.getPosToBone(ballCPos);
	Vector3f predictedBallPosToBone = WM.getPosToBone(ballPPos);
	Vector2f cBpos(currentBallPosToBone.x(),currentBallPosToBone.y());
	Vector2f pBPos(predictedBallPosToBone.x(),predictedBallPosToBone.y());
	//Line2f l(cBpos,pBPos);//line of currentBallPosToBone and predictedBallPosToBone
	Vector2f ballCPos2D(ballCPos.x(), ballCPos.y());
	AngDeg velAng = ballVel.angle();
	Line2f 	ballLine(ballCPos2D, velAng);
	AngDeg faceDirection = WM.getMyFaceDirection();
	Vector2f posMe = WM.getMyOrigin2D();
	const float boundary = (HUMANOID.getFootSize().y()*0.5f+ball_radius)+0.03f;
	Line2f meLine(posMe, faceDirection);
	Vector2f boundaryPos(posMe.x()+boundary*cosDeg(faceDirection), posMe.y()+boundary*sinDeg(faceDirection));
	Line2f tangentLine(boundaryPos, normalizeAngle(meLine.getAngle()-90.0f));

	Vector2f intersection = tangentLine.calIntersection(ballLine);

	Vector3f posTemp(intersection.x(), intersection.y(), ball_radius);

	Vector3f intersectionToBone = WM.getPosToBone(posTemp);

	float x= intersectionToBone.x();
	const float fall_bias = 0.135f;
	if(x<-fall_bias)
		return LEFT;
	else if(x>fall_bias)
		return RIGHT;
	else
		return UNKNOWN;
}

Direction Player::calFallDirection()
{
	//float x=ballPosToBone.x();
	Vector3f currentBallPosToBone=WM.getPosToBone(WM.getBallGlobalPos());
	Vector3f predictedBallPosToBone=WM.getPosToBone(mBallPredictedPos);
	Vector2f cBpos(currentBallPosToBone.x(),currentBallPosToBone.y());
	Vector2f pBPos(predictedBallPosToBone.x(),predictedBallPosToBone.y());
	Line2f l(cBpos,pBPos);//line of currentBallPosToBone and predictedBallPosToBone
	float y=HUMANOID.getFootSize().y()*0.5f+ball_radius;
	float x=l.calXGivenY(y);
	const float fall_bias = 0.135f;

	if(!isGoal())
		return UNKNOWN;
	else
	{
		//cout<<"should Fall"<<endl;
		if(x<-fall_bias)
			return LEFT;
		else if(x>fall_bias)
			return RIGHT;
		else
			return UNKNOWN;
	}
}

bool Player::isGoal()
{
	static bool isGoal[5]={0};
	for(int i=4;i>0;i--)
		isGoal[i]=isGoal[i-1];//record judements before
	bool isGoalTemp=false;

	// Direction fallDir;
	Vector3f ballVel=WM.getBallGlobalVel();
	Vector3f ballPos=WM.getBallGlobalPos();

	mBallPredictedPos=ballPos;//predicted postion of ball

	Vector3f ballToLeftGoal=Vector3f(-9.0f,half_goal_width,0)-ballPos;
	AngDeg angOfBallAndLeftGoal=ballToLeftGoal.angle();
	Vector3f ballToRightGoal=Vector3f(-9.0f,-half_goal_width,0)-ballPos;
	AngDeg angOfBallAndRightGoal=ballToRightGoal.angle();
	AngDeg angOfBallVel=ballVel.angle();
	bool isIn=isAngInInterval(angOfBallVel,angOfBallAndLeftGoal,angOfBallAndRightGoal);
	Vector3f tempBallVel=ballVel;

	for(int k=0;k<65;k++)
		WM.predictBall(mBallPredictedPos,tempBallVel);//predict postion of ball after 1.1 second

	Vector3f currentBallPosToBone=WM.getPosToBone(ballPos);
	Vector3f ballPosToBone=WM.getPosToBone(mBallPredictedPos);//get ballPredictedPos to bone

	float yCurrent=currentBallPosToBone.y();
	float y=ballPosToBone.y();

	if(isIn&&y<(HUMANOID.getFootSize().y()*0.5f+ball_radius))
	{
		isGoal[0]=true;
	}//judement of this period
	else
		isGoal[0] = false;

	int temp = 0;
	for (int j = 0; j < 5; j++)
	{
		if (isGoal[j])
			temp += 1;
		if (yCurrent > ((HUMANOID.getFootSize().y()*0.5f + ball_radius) + 0.03f) && temp >= 3)
			isGoalTemp = true;
	}// decide whether the goalKeeper should fall by curent ballPos and recorded judements
	return isGoalTemp;
}

bool Player::isBallWillCrossMeAfterNStps(int n)
{
	static float simTime; //this variable can make sure that we predict once every sim_step
	static bool isBallWillCrossMe[5] = {0};
	if (abs(simTime - WM.getSimTime()) > 0.001f)
	{
		for (int i = 4; i > 0; i--)
			isBallWillCrossMe[i] = isBallWillCrossMe[i - 1]; //record judements before

		Vector3f ballPPos = WM.getBallGlobalPos();
		Vector3f ballPVel = WM.getBallGlobalVel();
		Vector3f ballCPos = WM.getBallGlobalPos();
		for (int i = 0; i < n; i++)
			WM.predictBall(ballPPos, ballPVel);

		Vector3f currentBallPosToBone = WM.getPosToBone(ballCPos);
		Vector3f predictedBallPosToBone = WM.getPosToBone(ballPPos);

		float yCurrent = currentBallPosToBone.y();
		float yPredict = predictedBallPosToBone.y();

		const float boundary = (HUMANOID.getFootSize().y()*0.5f + ball_radius) + 0.03f;
		if ((yCurrent > boundary && yPredict < boundary)
				|| (yCurrent<-boundary && yPredict>-boundary))
			isBallWillCrossMe[0] = true;
		else
			isBallWillCrossMe[0] = false;
		simTime = WM.getSimTime();
	}//if we have not predict this sim_step,predict

	int temp = 0;
	for (int i = 0; i < 5; i++)
	{
		if (true == isBallWillCrossMe[i])
			temp += 1;
	}

	if (temp >= 3)
		return true;
	else
		return false;
}

/**
 * update GK state and return current state of GK
 */
GoalKeeperState Player::updateGoalKeeperState()
{
	GoalKeeperState state;
	mBalance.updateState();
	state = GoalKeeperState(mBalance.getCurrentState());
	return state;
}


//shared_ptr<Action> Player::goTo(const Vector2f& destPos, AngDeg bodyDir, bool avoidBall)
//{
////                       Vector2f posStop(destPos);
////                  math::AngDeg dir1 = WM.getMyBodyDirection();
////                Vector2f tar(posStop-WM.getMyGlobalPos2D());
////        //        cout << "golable "<<WM.getMyGlobalPos2D()  <<" dir1 "<< dir1 << " tar "<< tar <<  endl;
////             //   Vector2f xx(1,0);
////            //   dir = dir+ calClipAng(tar,x);
////                math::AngDeg fai = -WM.getMyBodyDirection()+90.0f;
////                Vector2f tarr(cosDeg(fai)*tar.x()-sinDeg(fai)*tar.y(),sinDeg(fai)*tar.x()+cosDeg(fai)*tar.y() );
////                dir1 = tarr.angle() - 90.0f;
////                normalizeAngle(dir1);
////             //   cout <<"calc"<< tar.angle() <<"\ndir2 "<< dir << endl;
////                return goToRel(tarr,dir1);
//
//	const Vector2f& curPos=WM.getMyGlobalPos2D();
//	AngDeg curBodyDir=WM.getMyBodyDirection(); //-180~180 deg
//
//	Vector2f deltaV2f=destPos-curPos;
//	AngDeg desiredWalkDir=deltaV2f.angle()-curBodyDir; ///////////////////////////
//	float dist=deltaV2f.length();
//
//	float x=-dist*sinDeg(desiredWalkDir);
//	float y= dist*cosDeg(desiredWalkDir);
//
//
//	AngDeg deltaBodyDir=normalizeAngle(bodyDir-curBodyDir);
//
//	return goToAvoidBlocks(Vector2f(x,y), deltaBodyDir, avoidBall);
//
//	/*//first should keep balance
//	shared_ptr<Action> act= mBalance.perform();
//	if ( NULL!=act.get() )
//	{
//		mKickLock=0;
//		mCameraMotionMode=0;
//		mTask.clear();
//		return act;
//	}
//
//	shared_ptr<Task> walkTask( new Walk(stopPos, dir, avoidBall, false) );
//	shared_ptr<Task> curTask= mTask.getFirstSubTask();
//
//	//there's no task, append the walk directly
//	if( NULL==curTask.get() )
//	{
//		mTask.append(walkTask);
//		return mTask.perform();
//	}
//
//	//if current task is WalkRel, tell it to stop
//	shared_ptr<WalkRel> curWalkRel= shared_dynamic_cast<WalkRel>(curTask);
//	if( NULL!=curWalkRel.get() )
//	{
//		curWalkRel->stopWalk();
//		if( mTask.getSubTaskListSize()<=1 ){ //TT: it's hard to say...
//			mTask.append(walkTask);
//		}
//		if( curWalkRel->getStepLength()<0.01f ){
//			mTask.clear();
//			mTask.append(walkTask);
//		}
//		return mTask.perform();
//	}
//
//	//...
//	shared_ptr<Walk> curWalk= shared_dynamic_cast<Walk>(curTask);
//	if( NULL==curWalk.get() ) //I am not walking, append the walk
//	{
//		if( mTask.getSubTaskListSize()<=1 )
//			mTask.append(walkTask);
//		return mTask.perform();
//	}
//	else //I am walking now, revise the current walk
//	{
//		curWalk->revise(walkTask);
//		return mTask.perform();
//	}*/
//}

shared_ptr<Action> Player::goTo(const Vector2f& destPos, AngDeg bodyDir, bool avoidBall)
{
	//WM.logPrintSeenFlags();
	
	Vector2f relTarget;
	float turnAng;
	Vector2f destRelPos=WM.transGlobalPosToRelPos( WM.getMyGlobalPos2D() , destPos );

	if( destRelPos.length()>0.2f ) //far enough, so don't care about body direction, just turn to destination
	{
		turnAng=-atan2Deg(destRelPos.x(),destRelPos.y());
		if( fabs(turnAng)>15.0f )
			relTarget=Vector2f(0,0);
		else
			relTarget=destRelPos;
	}
	else
	{
		relTarget=destRelPos;
		turnAng=normalizeAngle( bodyDir-WM.getMyBodyDirection() ); ///////////////////////////////
	}

		//return goToRel(relTarget,turnAng);
	return goToAvoidBlocks(relTarget, turnAng, avoidBall);
}

shared_ptr<Action> Player::goToRel(const Vector2f& target,AngDeg dir)
{
	//first should keep balance
	shared_ptr<Action> act= mBalance.perform();
	if ( NULL!=act.get() )
	{
		mKickLock=false;
		mCameraMotionMode=0;
		mTask.clear();
		return act;
	}

	shared_ptr<Task> walkTask( new WalkRel(target,dir) );
	shared_ptr<Task> curTask= mTask.getFirstSubTask();

	//there's no task, append the walk directly
	if( NULL==curTask.get() )
	{
		mTask.append(walkTask);
		return mTask.perform();
	}

	//if current task is Walk, tell it to stop
	shared_ptr<Walk> curWalk= shared_dynamic_cast<Walk>(curTask);
	if( NULL!=curWalk.get() )
	{
		curWalk->stopWalk();
		if( mTask.getSubTaskListSize()<=1 ){ //TT: maybe useless
			mTask.append(walkTask);
		}
		if( curWalk->getStepLength()<0.01f ){
			mTask.clear();
			mTask.append(walkTask);
		}
		return mTask.perform();
	}

	//...
	shared_ptr<WalkRel> curWalkRel= shared_dynamic_cast<WalkRel>(curTask);
	if( NULL==curWalkRel.get() ) //I am not walking, append the walk
	{
		if( mTask.getSubTaskListSize()<=1 )
			mTask.append(walkTask);
		return mTask.perform();
	}
	else //I am walking now, revise the current walk
	{
		curWalkRel->revise(walkTask);
		return mTask.perform();
	}
}


shared_ptr<Action> Player::goToSlow(const Vector2f& stopPos, AngDeg dir, bool avoidBall, bool is4Kick)
{
	// first should keep balance
	shared_ptr<Action> act = mBalance.perform();
	if (0 != act.get()) {
		mTask.clear();
		return act;
	}
	////terrymimi
	bool bull = false;
	//AngDeg face = WM.getMyFaceDirection();
	if (0 != isSpaceAhead(Vector2f(9, 0), 0.2))
	{
		bull = true;
	}

	shared_ptr<Task> walkTask(new WalkSlow(stopPos, dir, avoidBall));

	shared_ptr<Task> cTask = mTask.getFirstSubTask();
	if (NULL == cTask.get()) {
		// empty task, append the walk directly
		mTask.append(walkTask);
	}
	else {
		shared_ptr<WalkSlow> cWalk =
				shared_dynamic_cast<WalkSlow > (cTask);
		if (NULL == cWalk.get()) {
			// I am not walking currently, append the walk
			mTask.append(walkTask);
		}
		else {
			// I am walking now, revise the current walking
			cWalk->revise(walkTask);
		}
	}
	return mTask.perform();
}

shared_ptr<Action> Player::goTo(const Vector2f& stopPos, const Vector2f& lookAt, bool avoidBall)
{
	Vector2f v = lookAt - stopPos;
	return goTo(stopPos, v.angle(), avoidBall);
}

shared_ptr<Action> Player::goToSlow(const Vector2f& stopPos,
		const Vector2f& lookAt, bool avoidBall, bool is4Kick)
{
	Vector2f v = lookAt - stopPos;
	return goToSlow(stopPos, v.angle(), avoidBall);
}

shared_ptr<Action> Player::kickBetween(const Vector2f& goalLeft,const Vector2f& goalRight)
{
	const Vector2f& posBall = WM.getBallGlobalPos2D();
	Vector2f vBL = goalLeft - posBall;
	Vector2f vBR = goalRight - posBall;
	const AngDeg angLeft = vBL.angle();
	const AngDeg angRight = vBR.angle();
	const AngDeg angBisector = calBisectorTwoAngles(angRight, angLeft);
	Vector2f goal = posBall + pol2xyz(Vector2f(10.0f, angBisector));

	return kickTo(goal);
}

shared_ptr<Action> Player::walkToBall()
{
	return goTo(WM.getBallGlobalPos2D(),
			(WM.getBallGlobalPos2D() - WM.getMyOrigin2D()).angle());
}

/**
 * just beam to my initial position
 * and turn all joints to 'init' angle
 */
shared_ptr<Action> Player::beamAndInit(const Vector3f& beamPos)
{
	mBalance.perform();
	mBalance.reset();
	mTask.clear();

	shared_ptr<Action> bAct = shared_ptr<Action > (new BeamAction(beamPos));
	shared_ptr<Action> jAct = FAT.controlPreferThan("init", "*");

	Vector2f posMe, beamPos2D;
	posMe = WM.getMyGlobalPos();
	beamPos2D = beamPos;

	shared_ptr<Actions> acts = shared_ptr<Actions > (new Actions);
	if ((beamPos2D - posMe).length() < 0.05) {
		// if we have beam to the given position, stop beaming
		jAct = FAT.controlPreferThan("squat", "*");
		acts->add(jAct);
	}
	else {
		acts->add(bAct);
		acts->add(jAct);
	}

	return acts;
}

/******************************************Gravity test*************************************************************************/


shared_ptr<Action> Player::kickTo(const Vector2f& goal, bool useMaxForceMotion)
{
	shared_ptr<Action> act;

	//is kicking
	shared_ptr<Task> curTask=mTask.getFirstSubTask();
	shared_ptr<BasicKick> curKick=shared_dynamic_cast<BasicKick>(curTask);
	if( NULL!=curKick.get() )
	{
		act=mTask.perform();
		if( NULL!=act.get() )
		{
			mCameraMotionMode=0;
			return act;
		}
		else
		{
			mKickLock=false;
		}
	}

	//keep balance
	act=mBalance.perform();
	if( NULL!=act.get() )
	{
		mKickLock=false;
		mCameraMotionMode=0;
		mTask.clear();
		return act;
	}

	//choose kick type and calculate the target of walking
	Vector2f myDesiredPos;
	AngDeg myDesiredBodyDir;
	Vector2f relPosToStopWalk;
	shared_ptr<BasicKick> kick= chooseKickType(goal,useMaxForceMotion,&myDesiredPos,&myDesiredBodyDir,&relPosToStopWalk);

	if( (WM.getBallRelPos2D()+relPosToStopWalk).length()<0.013f ) //kick //0.01f 0.02f
	{
		shared_ptr<WalkRel> curWalkRel= shared_dynamic_cast<WalkRel>(curTask);
		if( NULL!=curWalkRel.get() )
		{
			curWalkRel->stopWalk();
			act=mTask.perform();
			if( NULL!=act.get() )
			{
				return act;
			}
		}
	//	printf("===========%s============\n","new KickTask in kickTo");
		mTask.clear();
		mTask.append(kick);
		return mTask.perform();
	}
	else if( (myDesiredPos-WM.getMyGlobalPos2D()).length()<0.07f //0.03f
			 && fabs(normalizeAngle(myDesiredBodyDir-WM.getMyBodyDirection()))<8.0f ) //return goToRel
	{
		return goToAvoidBlocks( WM.getBallRelPos2D()+relPosToStopWalk, 0, true );
	}
	else //return goTo
	{
		return goTo(myDesiredPos, myDesiredBodyDir, true);/////////////////////////////////
	}
}

boost::shared_ptr<task::BasicKick> Player::chooseKickType(const math::Vector2f& goal, bool useMaxForceMotion, Vector2f* pPos, AngDeg* pDir, Vector2f* pRelPosToStopWalk)
{
	//compare dist and choose a motion
	int i=0,motionNum=0; //select motionNum
	float targetDistToGoal=100.0f;
	float tempFloat;
	Vector2f kickTarget;

	if(useMaxForceMotion)
	{
		motionNum=0; //////////////////////////////////// which leg????????????????
	}
	else
	{
		FOR_EACH(iter,mKickMotionVector)
		{
			kickTarget=WM.transRelPosToGlobalPos( WM.getMyGlobalPos2D() , iter->kickTargetRel );
			tempFloat=(kickTarget-goal).length();
			if(tempFloat<targetDistToGoal){
				targetDistToGoal=tempFloat;
				motionNum=i;
			}
			i++;
		}
	}

	//////////////////////////////////////////////
	//motionNum=7;//////////////////////////////////////test

	//calculate pos and dir to go
	const Vector2f& ballPos=WM.getBallGlobalPos2D();
	Vector2f myDesiredPos=WM.transRelPosToGlobalPos( ballPos , mKickMotionVector[motionNum].myDesiredRelPosToBall );
	AngDeg myDesiredBodyDir= (goal-ballPos).angle() + atan2Deg(mKickMotionVector[motionNum].kickTargetRel.x(),
															   mKickMotionVector[motionNum].kickTargetRel.y()) ;
	*pPos=myDesiredPos;
	*pDir=myDesiredBodyDir;
	*pRelPosToStopWalk=mKickMotionVector[motionNum].relPosToBallToStopWalk;

	return shared_ptr<KickTask>( new KickTask(mKickMotionVector[motionNum].firstTaskName) );
}

Vector2f Player::chooseLargestAttackAngle(float x, float miny, float maxy, const Vector2f& pos, AngDeg& maxAng)
{
	const Vector2f left = Vector2f(x, maxy);
	const Vector2f right = Vector2f(x, miny);
	Vector2f pos2left = left - pos;
	Vector2f pos2right = right - pos;
	AngDeg angL = pos2left.angle();
	AngDeg angR = pos2right.angle();

	set<float> vec;
	vec.insert(miny);
	vec.insert(maxy);

	const map<unsigned int, Vector3f>& opp = WM.getOppGlobalPos();
	FOR_EACH(i, opp) {
		const Vector3f& p = i->second;
		if (p.x() > x) // skip outside opponent
			continue;
		if (p.x() < pos.x()) // skip opponent who is not behind ball
			continue;

		Vector2f pos2opp;
		pos2opp.x() = p.x() - pos.x();
		pos2opp.y() = p.y() - pos.y();
		AngDeg ang = pos2opp.angle();
		if (!isAngInInterval(ang, angR, angL)) // skip opponent who is not block
			continue;

		float y = (x - pos.x()) * tanDeg(ang) + pos.y();
		vec.insert(y);
	}

	float best = miny + maxy;
	Vector2f p1, p2;
	p1.x() = x;
	p2.x() = x;
	maxAng = 0;

	set<float>::const_iterator iter = vec.begin();
	p2.y() = *iter;
	AngDeg ang1, ang2;
	ang2 = (p2 - pos).angle();

	for (++iter; iter != vec.end(); ++iter) {
		p1.y() = p2.y();
		p2.y() = *iter;
		ang1 = ang2;
		ang2 = (p2 - pos).angle();
		AngDeg ang = calClipAng(ang2, ang1);
		if (ang > maxAng)
		{
			maxAng = ang;
			ang = calBisectorTwoAngles(ang2, ang1);
			best = (x - pos.x()) * tanDeg(ang) + pos.y();
		}
	}
	return Vector2f(x, best);
}

shared_ptr<Action> Player::shoot()
{
	if( WM.seenFlagsNum()>=3 )
	{
		const Vector2f& ballPos=WM.getBallGlobalPos2D();
		Vector2f goal(half_field_length,0);
		if( (goal-ballPos).length() > 2.3f )
		{
			return kickTo(goal,true);
		}
		else
		{
			return kickTo(goal,false);
		}
	}

	else
	{
		return dribbleToOppGoal();
	}
}

bool Player::chooseKickFoot(const Vector2f& goal)
{
	const Vector2f& ballPos = WM.getBallGlobalPos2D();
	Vector2f kickVec = goal - ballPos;
	AngDeg kickAng = kickVec.angle();
	AngDeg angMe2Ball = (ballPos - WM.getMyOrigin2D()).angle();
	AngDeg clipAng = calClipAng(angMe2Ball, kickAng);

	return clipAng>0;
}

bool Player::chooseKickFoot2(const math::Vector2f& goal)
{
	const Vector2f ballPos2D = WM.getBallGlobalPos2D();
	const Vector2f ball2Goal = goal - ballPos2D;
	Vector3f temp = WM.getBoneTrans(HUMANOID.L_FOOT).pos();
	Vector2f leftFootPos(temp.x(), temp.y());
	temp = WM.getBoneTrans(HUMANOID.R_FOOT).pos();
	Vector2f rightFootPos(temp.x(), temp.y());

	return (ballPos2D - leftFootPos).length() < (ballPos2D - rightFootPos).length();
}

shared_ptr<action::Action> Player::defense()
{
	const Vector3f& oppPos = WM.getOppFastestToBallPos();
	const Vector2f& posMe = WM.getMyOrigin2D();
	const Vector2f& posBall = WM.getBallGlobalPos2D();
	float distOpp = sqrt(pow2(oppPos.x() - posBall.x()) + pow2(oppPos.y() - posBall.y()));
	float distMe = (posBall - posMe).length();
	if ((distOpp < distMe && WM.getMyInterceptBallTime() > 2 && WM.howIFasterToBallThanOpps() < -2) && WM.getBallGlobalPos()[0] <= 1.0f) {
		return runDefensePos();
	}
	else {
		return clearBall();
	}
}

shared_ptr<Action> Player::runDefensePos()
{
	const Vector2f& posBall = WM.getBallGlobalStopPos2D(); //WM.getBallGlobalPos2D();
	float goaly =0 ;
	switch (FM.getMy().type)
	{
		case configuration::Formation::PT_MIDFIELDER_CENTER :
			goaly =0 ;
			break;
		case configuration::Formation::PT_DEFENDER_CENTRAL :
			goaly =0.5;
			break;
		case configuration::Formation::PT_DEFENDER_SWEEPER :
		case configuration::Formation::PT_DEFENDER_WING :
			goaly =-0.5;
			break;
		default:
			goaly =0;
			break;
	}
	Vector2f ourGoal(-half_field_length, goaly);
	Vector2f p = posBall - ourGoal;
	float len = p.length();
	float dist = len * 0.618f;
	dist = max(dist, 1.5f);
	dist = min(dist, len);
	p.normalize();
	p *= dist;
	p += ourGoal;
	return goTo(p, posBall);
}

/**
 * for test
 */
shared_ptr<action::Action> Player::randomBeam()
{
	return shared_ptr<Action>(new BeamAction(Vector3f(int(random(-5.5,-0.5)*10) / 10.0f,
													  int(random(-3.5,3.5)*10) / 10.0f,
													  0)));
}

boost::shared_ptr<action::Action> Player::dribbleToDir(AngDeg angC, AngDeg angL, AngDeg angR)
{
	const Vector2f& ballPos=WM.getBallGlobalPos2D();
	const Vector2f& myPos=WM.getMyGlobalPos2D();

	AngDeg myDirToBall=(ballPos-myPos).angle();

	if( isAngInInterval(myDirToBall,angR,angL) ) //TT note: angR<angL
	{
		return dribbleRel();
	}
	else
	{
		float dist= min( 0.3f , WM.getBallPol2D().x() ) ; //distance to ball
		float x=ballPos.x()-dist*cosDeg(angC);
		float y=ballPos.y()-dist*sinDeg(angC);
		return goTo( Vector2f(x,y), angC , true );
	}
}

boost::shared_ptr<action::Action> Player::dribble()
{
	if( WM.seenFlagsNum()>=3 )
	{
		const Vector2f& myPos=WM.getMyGlobalPos2D();
		const Vector2f& ballPos=WM.getBallGlobalPos2D();
		Vector2f target(half_field_length,0);

		//calculate L and R
		Vector2f pointL(half_field_length, half_goal_width);
		Vector2f pointR(half_field_length,-half_goal_width);
		AngDeg angC= (target-ballPos).angle();
		AngDeg angL= (pointL-ballPos).angle();
		AngDeg angR= (pointR-ballPos).angle();

		float ballDistToTarget=(target-ballPos).length();

		if( ballDistToTarget<4.0f ) //near enough to the target
		{
			return dribbleToDir(angC, angL, angR);
		}
		else
		{
			//=================enlarge the range
			float k=0.96f; ///////////////////////////////////// 0.48f
			float deltaDist= ballDistToTarget-4.0f ;
			//L
			float deltaAngL= angL-angC ;
			deltaAngL+= deltaAngL * k * deltaDist ;
			angL= normalizeAngle(angC+deltaAngL) ;
			//R
			float deltaAngR= angC-angR ;
			deltaAngR+= deltaAngR * k * deltaDist ;
			angR= normalizeAngle(angC-deltaAngR) ;

			//=================restrict L and R according to the court info
			//...
			return dribbleToDir(angC, angL, angR);
		}
	}
	else
	{
		return dribbleToOppGoal();
	}
}

boost::shared_ptr<action::Action> Player::clearBall()
{
	if(WM.seenFlagsNum()>=2)
	{
		const Vector2f& ballPos=WM.getBallGlobalPos2D();
		Vector2f goal(-half_field_length,0);
		float ballDistToGoal= (ballPos-goal).length();
		AngDeg goalAngToBall= (ballPos-goal).angle();
		AngDeg myDirToBall= ( ballPos - WM.getMyGlobalPos2D() ).angle();

		if( fabs(myDirToBall) < max(70.0f,fabs(goalAngToBall)) )
		{
			return kickRel();
		}
		else
		{
			float myDesiredDistToBall= min( 0.6f, WM.getBallPol2D().x() );
			Vector2f myDesiredPos= ballPos - myDesiredDistToBall/ballDistToGoal*(ballPos-goal) ;
			return goTo(myDesiredPos, goalAngToBall, true);
		}
	}

	else
	{
		if( WM.getBallGlobalPos2D().x() > WM.getMyGlobalPos2D().x() ) /////////////////////////////////
			return dribbleRel();
		else
			return dribbleToOppGoal(); /////////////////////////////////////////////////////////////
	}
}

    boost::shared_ptr<action::Action> Player::pass(const bool walkSlow) {
        static Vector2f lastPassPoint(WM.getBestZoneX()*18 / 9 + 1 - 9, WM.getBestZoneY()*12 / 6 + 1 - 6);
        return kickTo(lastPassPoint);
    }

    int Player::isSpaceAhead(const Vector2f& goal, float dist) {
        const Vector2f& posBall = WM.getBallGlobalPos2D();
        const Vector2f& posMe = WM.getMyOrigin2D();
        AngDeg angThr = clamp((field_length - 2.0f * posBall.x())*30.0f / field_length, 30.0f, 60.0f);
        AngDeg dir1 = (goal - posMe).angle();

        AngDeg angL = normalizeAngle(dir1 + angThr);
        AngDeg angR = normalizeAngle(dir1 - angThr);

        const map<unsigned int, Vector3f>& oppPos = WM.getOppGlobalPos();
        Vector2f vec;

        FOR_EACH(iter, oppPos) {
            vec.x() = iter->second.x() - posMe.x();
            vec.y() = iter->second.y() - posMe.y();
            float distOpp = vec.length();
            AngDeg angOpp = vec.angle();

            if (distOpp < dist && isAngInInterval(angOpp, angR, angL)) {
                return iter->first;
            }
        }
        return 0;
    }

    Vector2f Player::calPassPoint(unsigned int num) {
        const Vector2f goal(half_field_length, 0);
        Vector2f posTeammate = WM.getOurGlobalPos2D(num);
        const Vector2f& posBall = WM.getBallGlobalPos2D();
        float pass_dist = 5;//4
        float minX = max(-half_field_length, posBall.x() - pass_dist);
        minX = max(minX, posTeammate.x() - 1);
        float maxX = min(half_field_length, posBall.x() + pass_dist);
        float minY = max(-half_field_width, posBall.y() - pass_dist);
        float maxY = min(half_field_width, posBall.y() + pass_dist);
        posTeammate += (goal - posTeammate).normalized() * 0.5f;

        const Vector2f plb(minX, minY);
        const Vector2f prb(maxX, minY);
        const Vector2f prt(maxX, maxY);
        const Vector2f plt(minX, maxY);

        // the polygon of the soccer field
        TVoronoi<float> voronoi;
        voronoi.create(plb, prb, prt, plt);

        if (!voronoi.setInnerPoint(posTeammate)) {
            return posTeammate;
        }

        Vector2f p;
        const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();

        FOR_EACH(iter, ourPos) {
            if (iter->first == num) continue;
            p.x() = iter->second.x();
            p.y() = iter->second.y();
            Line2f line = Line2f::makeMidperpendicularFromTwoPoints(p, posTeammate);
            voronoi.cutByLine(line);
        }

        const map<unsigned int, Vector3f>& oppPos = WM.getOppGlobalPos();

        FOR_EACH(iter, oppPos) {
            p.x() = iter->second.x();
            p.y() = iter->second.y();
            Line2f line = Line2f::makeMidperpendicularFromTwoPoints(p, posTeammate);
            voronoi.cutByLine(line);
        }
        return voronoi.calCentroid();
    }

    bool Player::isPassingToMe() {
        static float lastTime = 0;
        const vector<shared_ptr<perception::Hear> >& hear = WM.lastPerception().hear();

        FOR_EACH(iter, hear) {
            if ((*iter)->isSelfMsg())
                continue;

            const string& msg = (*iter)->message();
            if (string::npos != msg.find("->")) {
                stringstream ss;
                ss << msg.substr(msg.find("->") + 2);
                unsigned int num = 0;
                ss >> num;
                if (num == WM.getMyUnum()) {
                    lastTime = WM.getSimTime();
                    return true;
                }
            }
        }

        return WM.getSimTime() - lastTime < sim_step * 3;
    }

    bool Player::amIFreeToShoot(int num) {
        int dist = 2;
        Vector2f vec(0, 0);
        Vector2f posMe = WM.getMyGlobalPos2D();
        const map<unsigned int, Vector3f>& oppPos = WM.getOppGlobalPos();

        FOR_EACH(iter, oppPos) {
            vec.x() = iter->second.x() - posMe.x();
            vec.y() = iter->second.y() - posMe.y();
            float distOpp = vec.length();
            AngDeg angOpp = vec.angle();

            if (distOpp < dist) {
                return iter->first;
            }
        }
    }

    configuration::Formation::FormationType Player::chooseFormation() {
        int opphalfnum = 0;
        int oppdefnum = 0;
        int opp2num = 0;
        const map<unsigned int, Vector3f>& oppPos = WM.getOppGlobalPos();

        FOR_EACH(iter, oppPos) {
            if (iter->second.x() > 0) {
                opphalfnum++;
            }
            if (iter->second.x() > 6.5) {
                oppdefnum++;
            }
            if (iter->second.x()<-2) {
                opp2num++;
            }
        }

        if (opp2num > 2 || opphalfnum <= 3) {
            return configuration::Formation::FT_ATTACK_MIDDLE;
        } else if (oppdefnum >= 5) {
            return configuration::Formation::FT_DEFEND_MIDDLE;
        } else {
            return configuration::Formation::FT_HOME;
        }
    }

shared_ptr<Action> Player::goToAvoidBlocks(math::Vector2f dest, math::AngDeg bodyDir, bool avoidBall)
{
	float walkDir=-atan2Deg(dest.x(),dest.y());

	const std::list<core::WorldModel::BlockInfo>& blockList=WM.getBlockList();
	const core::WorldModel::BlockInfo& ballBlock=WM.getBallBlock();

	float nearestPlayerBlockDist=100.0f; //used in judgement for ball

	//ball
	if( avoidBall && WM.canSeeBall() && ballBlock.dist<nearestPlayerBlockDist ) //can see?????????????????
	{
		do
		{
			if( (ballBlock.dist)>dest.length()/**0.9f*/ )
				break;

			/*||walkDir ballBlock.angC*/

			if( false==isAngInInterval(walkDir,ballBlock.angR,ballBlock.angL) )
				break;

			//the ball blocks me
			walkDir= isAngInInterval(walkDir,ballBlock.angR,ballBlock.angC) ? ballBlock.angR : ballBlock.angL ;
			dest.x()=-ballBlock.dist * sinDeg(walkDir) ;
			dest.y()= ballBlock.dist * cosDeg(walkDir) ;

		} while(0);
	}

	return goToRel(dest,bodyDir); //bodyDir, instead of walkDir
}

shared_ptr<Action> Player::kickRel()
{
	shared_ptr<Action> act;

	//is kicking
	shared_ptr<Task> curTask=mTask.getFirstSubTask();
	shared_ptr<BasicKick> curKick=shared_dynamic_cast<BasicKick>(curTask);

	if( NULL!=curKick.get() )
	{
		//TT test
		/*perception::JointPerception jp=WM.lastPerception().joints();
		for(int i=0;i<22;i++)
			printf("%.2f\t",jp[i].angle());
		printf("\n");*/
		///////////////////////////////////////////////////

		act=mTask.perform();

		if( NULL!=act.get() )
		{
			mCameraMotionMode=0;
			return act;
		}
		else
		{
			mKickLock=false;
		}
	}

	//keep balance
	act=mBalance.perform();
	if( NULL!=act.get() )
	{
		mKickLock=false;
		mCameraMotionMode=0;
		mTask.clear();
		return act;
	}

	//get closer to the ball
	Vector2f tempV2f(0,0);
	float dir=0.0f;
	const Vector2f& ballRelPos=WM.getBallRelPos2D();
	bool useRightFoot= ( ballRelPos.x() > 0 ) ;

	if(ballRelPos.y()<0.01f) //turn body
	{
		if(ballRelPos.x()<0) dir=20;
		else dir=-20;
	}
	else
	{
		if( ballRelPos.length()>0.3f )
			dir=(-1)*atan2Deg(ballRelPos.x(),ballRelPos.y());

		if(fabs(dir)<15.0f)
			tempV2f= ballRelPos + Vector2f( (useRightFoot ? -0.07f : +0.07f) ,-0.19f) ;//-0.055f -0.19f
	}

	//decide to walk or kick
	if( tempV2f.length()>0.013f || fabs(dir)>8.0f ) //////////////////////////////////// 0.01f
	{
		act=goToRel(tempV2f,dir);
		return act;
	}
	else
	{
		shared_ptr<WalkRel> curWalkRel= shared_dynamic_cast<WalkRel>(curTask);
		if( NULL!=curWalkRel.get() )
		{
			curWalkRel->stopWalk();
			act=mTask.perform();
			if( NULL!=act.get() )
			{
				return act;
			}
		}

		printf("===========%s============\n","new kick in kickRel");

		//shared_ptr<BasicKick> kick=chooseKickType(Vector2f(0,0));
		shared_ptr<BasicKick> kick= useRightFoot ? shared_ptr<KickTask>(new KickTask("BTTL_t1")) :
												   shared_ptr<KickTask>(new KickTask("BTT_t1")) ;
		//shared_ptr<BasicKick> kick= shared_ptr<Shoot>( new Shoot(Vector2f(0,0)) );

		//printf("size=%d\n",mTask.getSubTaskListSize()); //1 or sometimes 0
		mTask.clear();
		//printf("size=%d\n",mTask.getSubTaskListSize()); //0
		mTask.append(kick);
		//printf("size=%d\n",mTask.getSubTaskListSize()); //1
		act=mTask.perform();
		//printf("size=%d\n",mTask.getSubTaskListSize()); //should be 1
		return act;
	}
}

    unsigned int Player::choosePassPlayer() {

        const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();
        Vector2f myPos = WM.getMyGlobalPos2D();
        Vector2f ballPos = WM.getBallGlobalPos2D();
        unsigned int bestid = 0;
        if (!ourPos.empty()) {

            FOR_EACH(iter, ourPos) {
                if (ballPos.x() < iter->second.x()) {
                    if ((ballPos - WM.getOurGlobalPos2D(iter->first)).length() > 2 && iter->first != WM.getMyUnum()) {
                        bestid = iter->first;
                    }
                }
            }
        }
        return bestid;
    }

    boost::shared_ptr<action::Action> Player::passToPlayer(unsigned int num) {

        Vector2f globalTar(PM.getOurTeammates()[num].getMGlobalPosAll2D());
//        Vector2f myglo = WM.getMyGlobalPos2D();
//        Vector2f goal(half_field_length,0);
//     //   Vector2f left;
//     //   Vector2f right;
//     //   left = (globalTar-myglo).
//      return  kickToBetweenRel(GlobalToRel(globalTar),GlobalToRel(goal));
        globalTar.x() = globalTar.x()+0.5;
        if(globalTar.y()>WM.getBallGlobalPos().y()){
        globalTar.y() = globalTar.y()-0.5;
        }else{
            globalTar.y()= globalTar.y()+0.5;
        }
        return kickTo(globalTar);
//        globalTar = globalTar - WM.getMyGlobalPos2D();
//        math::AngDeg fai1 = -WM.getMyBodyDirection() + 90.0f;
//        Vector2f kickToTarget(cosDeg(fai1) * globalTar.x() - sinDeg(fai1) * globalTar.y(), sinDeg(fai1) * globalTar.x() + cosDeg(fai1) * globalTar.y());
        Vector2f relpos(PM.getOurTeammates()[num].getRelPos().x(),PM.getOurTeammates()[num].getRelPos().y());

        //   cout << "pass tar " << kickToTarget << endl;
        return kickToRel(relpos);

    }

    math::Vector2f Player::GlobalToRel(const math::Vector2f& global) {

        Vector2f globalTar;
        Vector2f kickToTarget;
        globalTar = global;
        globalTar = globalTar - WM.getMyGlobalPos2D();
        math::AngDeg fai1 = -WM.getMyBodyDirection() + 90.0f;
        kickToTarget.set(cosDeg(fai1) * globalTar.x() - sinDeg(fai1) * globalTar.y(), sinDeg(fai1) * globalTar.x() + cosDeg(fai1) * globalTar.y());
        //   cout << "global reee" << global <<"globalTar" << globalTar<< endl;
        //    cout << "global reee" << kickToTarget<<endl;
        return kickToTarget;

    }

    boost::shared_ptr<action::Action> Player::interceptionBall() {
        const Vector2f& vel = WM.getBallRelVel2D();
        const Vector2f& ball = WM.getBallRelPos2D();
        Vector2f yuanDi(0.0f, 0.0f);
        if (vel.angle()<-45 && vel.angle()>-135 || vel.angle() > 45 && vel.angle() < 135) {
            float tarX = -(vel.x() / vel.y()) * ball.y() + ball.x();
            if (tarX > 0.1) {
                return sideWalk(false);
            } else if (tarX<-0.1) {
                return sideWalk(true);
            } else {
                return goToRel(yuanDi, 0);
            }
        }
        return goToRel(yuanDi, 0);
    }

    boost::shared_ptr<action::Action> Player::sideWalk(bool isLeft) {
        if (isLeft) {
            Vector2f tar(-0.15, 0);
            return goToRel(tar, 0);
        } else {
            Vector2f tar(0.15, 0);
            return goToRel(tar, 0);
        }

    }
    //targetRel为相对于自身的坐标

    boost::shared_ptr<action::Action> Player::kickToRel(const math::Vector2f targetRel) {
        //   Vector2f globalTar(half_field_length, 0);
        //      math::AngDeg fai1 = -WM.getMyBodyDirection()+90.0f;
        Vector2f kickToTarget(targetRel);
        Vector2f kickBallPos = WM.getBallRelPos2D();
        Vector2f goToTarget1(kickToTarget - kickBallPos);
        Vector2f goToTarget(kickBallPos.x() - 0.3f * goToTarget1.x() / (goToTarget1.x() + goToTarget1.y()), kickBallPos.y() - 0.3f * goToTarget1.y() / (goToTarget1.x() + goToTarget1.y()));
        //    cout <<"goTotar" << goToTarget <<endl;
        math::AngDeg bodyDir = kickToTarget.angle() - 90.0f;
        //    math::AngDeg bodyToBallDir = kickBallPos.angle() - 90.0f;
        //    cout<<"bodyDir"<<bodyDir<<endl;
        //     cout<<"WM.getMyBodyDirection()"<<WM.getMyBodyDirection()<<endl;
        //      cout <<"getFlagNumbersISee()"<<WM.getFlagNumbersISee()<<endl;
        if (abs(bodyDir) > 10 || abs(goToTarget.x()) > 1 || abs(goToTarget.y()) > 1) {
            //    globalTar.x() =8.6f;
            //    globalTar.y()  = 3.0f;
            //    Vector2f kickToTarget1(cosDeg(fai1)*globalTar.x()-sinDeg(fai1)*globalTar.y(),sinDeg(fai1)*globalTar.x()+cosDeg(fai1)*globalTar.y() );

            return goToRel(goToTarget, bodyDir);
            //   return goTo(globalTar,0,true,true);
        }
        return kickRel();
    }
    //leftBoundary,  rightBoundary 为相对于自身的坐标

    boost::shared_ptr<action::Action> Player::kickToBetweenRel(const math::Vector2f& leftBoundary, const math::Vector2f& rightBoundary) {
        static Vector2f globalKickToTar(0.0f, 0.0f);
        Vector2f globalBallPos = WM.getBallGlobalPos2D();
        Vector2f kickToTar(GlobalToRel(globalKickToTar));
        math::AngDeg leftDir = leftBoundary.angle() - 90.0f;
        math::AngDeg rightDir = rightBoundary.angle() - 90.0f;
        math::AngDeg tarDir = kickToTar.angle() - 90.0f;
        Vector2f kickBallPos = WM.getBallRelPos2D();

        //cout << "globalKickToTar" << globalKickToTar << endl;
        if ((kickToTar - kickBallPos).length() < 0.5 || (abs(tarDir) > abs((leftDir - rightDir)*0.3))) {
            //cout << "change" << endl;
            globalKickToTar.x() = (leftBoundary.x() + rightBoundary.x()) / 2;
            globalKickToTar.y() = (leftBoundary.y() + rightBoundary.y()) / 2;
            math::AngDeg fai1 = 360 - (-WM.getMyBodyDirection() + 90.0f);
            Vector2f globalTar(globalKickToTar);
            globalKickToTar.set(cosDeg(fai1) * globalTar.x() - sinDeg(fai1) * globalTar.y(), sinDeg(fai1) * globalTar.x() + cosDeg(fai1) * globalTar.y());

            globalKickToTar += WM.getMyGlobalPos2D();

            kickToTar = GlobalToRel(globalKickToTar);

            //            float dir = ((leftBoundary - kickBallPos).angle() + (rightBoundary - kickBallPos).angle()) / 2;
            //            kickToTar = WM.getBallRelPos2D() + pol2xyz(Vector2f(3, dir));
        }
        return kickToRel(kickToTar);

    }

    boost::shared_ptr<action::Action> Player::breakingBall() {
        Vector2f ballRel = WM.getBallRelPos2D();
        AngDeg dir = ballRel.angle() - 90.0f;
        Vector2f target(ballRel.x(), ballRel.y() + 0.1f);
        return goToRel(target, dir);
    }

    shared_ptr<Action> Player::shootRel() {
//        math::Vector2f goal(half_field_length,0.0f);
//        return kickTo(goal);
        mCameraMotionMode = 4;

        //======================================================kicking
        shared_ptr<Task> cTask = mTask.getFirstSubTask();
        shared_ptr<BasicKick> cKick = shared_dynamic_cast<BasicKick > (cTask);
        if (NULL != cKick.get()) {
            //printf("%s\n","NULL!=cKick.get()");
            shared_ptr<Action> act = mTask.perform();

            if (NULL != act.get()) {
                mCameraMotionMode = 0;
                return act;
            } else {
                //cout<<"kick done"<<endl;
                mKickLock = false;
            }
        }

        //================================================keep balance
        shared_ptr<Action> actKB = mBalance.perform();
        if (NULL != actKB.get()) {
            mKickLock = false;
            mCameraMotionMode = 0;
            mTask.clear();
            return actKB;
        }


        //=================================================info
        const Vector2f& myPos = WM.getMyGlobalPos2D();

        bool seeBall = WM.canSeeBall();
        const Vector2f& ballRelPos = WM.getBallRelPos2D();
        const Vector2f& ballPos = WM.getBallGlobalPos2D();
        bool seeGoalL = WM.canSeeFlag(Vision::G1R);
        bool seeGoalR = WM.canSeeFlag(Vision::G2R);
        const Vector2f& goalLRel = WM.getFlagRelPos2D(Vision::G1R);
        const Vector2f& goalRRel = WM.getFlagRelPos2D(Vision::G2R);

        float myDistToBall = WM.getBallPol2D().x();

        float myDistToGoal;
        if (seeGoalL)
            myDistToGoal = goalLRel.length();
        else if (seeGoalR)
            myDistToGoal = goalRRel.length();
        else
            myDistToGoal = (Vector2f(half_field_length, 0) - myPos).length();

        //=================================================special conditions, give up kicking
        if (myDistToBall > 0.4f) {
            mKickLock = false;
        }


        //=================================================return kickRel if I decided to kick
        if (true == mKickLock) {
            return kickRel();
        }


        //==================================================decide to kick when me-ball-goal 3 points on a same line
        if (WM.seenFlagsNum() >= 2 && seeBall) {
            float y = (ballPos.y() - myPos.y()) * (half_field_length - myPos.x()) / (ballPos.x() - myPos.x()) + myPos.y();
            float range = 0.3f; //////////////////////////////////////
            if (myDistToGoal > 5)
                range += 0.0156f * (myDistToGoal * myDistToGoal - 25);

            float delta = y - 0.3f; ///////////////////////////////////////////////
            bool isBehindBall = myPos.x() < ballPos.x();

            if (isBehindBall) {
                if (fabs(delta) < range) {
                    mKickLock = true;
                    return kickRel();
                } 
            }
        }
      //  return goToRel(WM.getBallRelPos2D(),WM.getBallRelPos2D().angle()-90);

        //=====================================
        return goToBallBack();
        //=====================================
    }

shared_ptr<Action> Player::dribbleRel()
{
	const Vector2f& ballRelPos=WM.getBallRelPos2D();
	float dir=WM.getBallPol2D().y();

	//for dribble FORWARD
	Vector2f adjustV2f( (ballRelPos.x()>0 ? -0.05f : 0.05f) , 0.05f );//////////////////////////////
	//===========

	return goToRel( ballRelPos+adjustV2f , dir );
}

shared_ptr<Action> Player::dribbleToOppGoal()
{
	mCameraMotionMode=4;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	float angBall=WM.getBallPol2D().y();
	float angGoal1=WM.getFlagPol2D(Vision::G1R).y();
	float angGoal2=WM.getFlagPol2D(Vision::G2R).y();

	//=======================
	Vector2f goalRelPos;
	float ballDistToGoal;
	float k=0.96f; ////////////////////////////////////////
	float angRange=0;

	if( WM.canSeeFlag(Vision::G1R) && WM.canSeeFlag(Vision::G2R) )
	{
		goalRelPos= Vector2f( WM.getFlagRelPos2D(Vision::G1R) + WM.getFlagRelPos2D(Vision::G2R) ) / 2 ;
		ballDistToGoal= ( goalRelPos - WM.getBallRelPos2D() ).length();
		angRange= k * ballDistToGoal ;
		if( (angGoal1+angRange)>angBall && angBall>(angGoal2-angRange) )
			return dribbleRel();
		else
			return goToBallBack();
	}

	else if( WM.seenFlagsNum()>=3 && WM.canSeeBall() ) //use global info
	{
		Vector2f deltaV2f=Vector2f(half_field_length,0)-WM.getMyGlobalPos2D();
		float angG=deltaV2f.angle() - WM.getMyBodyDirection();
		float myDistToGoal=deltaV2f.length();
		float xg=-myDistToGoal*sinDeg(angG);
		float yg= myDistToGoal*cosDeg(angG);
		goalRelPos=Vector2f(xg,yg);

		ballDistToGoal= ( goalRelPos - WM.getBallRelPos2D() ).length();
		angRange= k * ballDistToGoal ;
		if( (angGoal1+angRange)>angBall && angBall>(angGoal2-angRange) )
			return dribbleRel();
		else
			return goToBallBack();
	}

	else if( WM.canSeeFlag(Vision::G1R) )
	{
		goalRelPos= WM.getFlagRelPos2D(Vision::G1R) ;
		ballDistToGoal= ( goalRelPos - WM.getBallRelPos2D() ).length();
		angRange= k * ballDistToGoal ;
		if( (angGoal1+angRange)>angBall ) ////////////////////////////////////////
			return dribbleRel();
		else
			return goToBallBack();
	}

	else if( WM.canSeeFlag(Vision::G2R) )
	{
		goalRelPos= WM.getFlagRelPos2D(Vision::G2R) ;
		ballDistToGoal= ( goalRelPos - WM.getBallRelPos2D() ).length();
		angRange= k * ballDistToGoal ;
		if( angBall>(angGoal2-angRange) ) ////////////////////////////////////////
			return dribbleRel();
		else
			return goToBallBack();
	}

	else
	{
		return goToBallBack();
	}
}

shared_ptr<Action> Player::goToBallBack()
{
	mCameraMotionMode=4;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	float xb=WM.getBallRelPos2D().x();
	float yb=WM.getBallRelPos2D().y();
	float angB=WM.getBallPol2D().y();
	float myDistToBall=WM.getBallPol2D().x();
	float xg=0,yg=0,angG=0;
	float x=0,y=0,ang=0;

	//========================info
	if( WM.canSeeFlag(Vision::G1R) && WM.canSeeFlag(Vision::G2R) )
	{
		xg=( WM.getFlagRelPos2D(Vision::G1R).x() + WM.getFlagRelPos2D(Vision::G2R).x() ) / 2 ;
		yg=( WM.getFlagRelPos2D(Vision::G1R).y() + WM.getFlagRelPos2D(Vision::G2R).y() ) / 2 ;
		angG=-atan2Deg(xg,yg); //TT: this is accurate
	}

	else if( WM.seenFlagsNum()>=3 && WM.canSeeBall() )
	{
		//don't want to use global position
		Vector2f deltaV2f=Vector2f(half_field_length,0)-WM.getMyGlobalPos2D();
		angG=deltaV2f.angle() - WM.getMyBodyDirection();
		float myDistToGoal=deltaV2f.length();
		xg=-myDistToGoal*sinDeg(angG);
		yg=myDistToGoal*cosDeg(angG);
	}

	else if( WM.canSeeFlag(Vision::G1R) )
	{
		xg=WM.getFlagRelPos2D(Vision::G1R).x();
		yg=WM.getFlagRelPos2D(Vision::G1R).y();
		angG=WM.getFlagPol2D(Vision::G1R).y();
	}

	else if( WM.canSeeFlag(Vision::G2R) )
	{
		xg=WM.getFlagRelPos2D(Vision::G2R).x();
		yg=WM.getFlagRelPos2D(Vision::G2R).y();
		angG=WM.getFlagPol2D(Vision::G2R).y();
	}

	else
	{
		//mCameraMotionMode=3; //search flags
		return goToRel( WM.getBallRelPos2D() , -atan2Deg(xb,yb) ); /////////////////////////////////////
	}


	//======================================
	//get closer to the ball
	if(myDistToBall>1.5f)
	{
		float D=min(0.7f,myDistToBall); //desired distance to ball
		float d=sqrt( (xb-xg)*(xb-xg) + (yb-yg)*(yb-yg) );
		if(d<EPSILON) d=EPSILON;

		x=(xb-xg)*D/d+xb;
		y=(yb-yg)*D/d+yb;
		ang=-atan2Deg(x,y);

		if(ang>90.0f) ang-=180.0f;
		else if(ang<-90.0f) ang+=180.0f;

		if( fabs(ang)<30.0f )
			return goToAvoidBlocks(Vector2f(x,y),ang,true);
		else
			return goToRel(Vector2f(0,0),ang);
	}
	else
	{
		float D=min(0.3f,myDistToBall); //desired distance to ball
		float d=sqrt( (xb-xg)*(xb-xg) + (yb-yg)*(yb-yg) );
		if(d<EPSILON) d=EPSILON;

		x=(xb-xg)*D/d+xb;
		y=(yb-yg)*D/d+yb;
		ang=angG;

		return goToAvoidBlocks(Vector2f(x,y),ang,true);
	}
}


shared_ptr<Action> Player::fallToGetBall(int dir)
{
	shared_ptr<Action> act;
	shared_ptr<Task> curTask=mTask.getFirstSubTask();
	shared_ptr<BasicKick> curKick=shared_dynamic_cast<BasicKick>(curTask);

	if( NULL!=curKick.get() )
	{
		act=mTask.perform();

		if( NULL!=act.get() )
		{
			mCameraMotionMode=0;
			return act;
		}
		else
		{
			;
		}
	}

	//keep balance
	act=mBalance.perform();
	if( NULL!=act.get() )
	{
		mKickLock=false;
		mCameraMotionMode=0;
		mTask.clear();
		return act;
	}

	//new fall
	shared_ptr<WalkRel> curWalkRel= shared_dynamic_cast<WalkRel>(curTask);
	if( NULL!=curWalkRel.get() )
	{
		curWalkRel->stopWalk();
		act=mTask.perform();
		if( NULL!=act.get() )
		{
			return act;
		}
	}
	if(-1==dir)
	{
		shared_ptr<BasicKick> kick= shared_ptr<KickTask>(new KickTask("leftfall_pt_init_squat"));
		mTask.clear();
		mTask.append(kick);
		act=mTask.perform();
		return act;
	}
	else if(1==dir)
	{
		shared_ptr<BasicKick> kick= shared_ptr<KickTask>(new KickTask("rightfall_pt_init_squat"));
		mTask.clear();
		mTask.append(kick);
		act=mTask.perform();
		return act;
	}
	else if(-2==dir)
	{
		shared_ptr<BasicKick> kick= shared_ptr<KickTask>(new KickTask("LFToLie1_wcy"));
		mTask.clear();
		mTask.append(kick);
		act=mTask.perform();
		return act;
	}
	else if(2==dir)
	{
		shared_ptr<BasicKick> kick= shared_ptr<KickTask>(new KickTask("RFToLie1_wcy"));
		mTask.clear();
		mTask.append(kick);
		act=mTask.perform();
		return act;
	}
}


     shared_ptr<Action> Player::testActionTT() {
 return kickTo(WM.getBallGlobalPos2D() + Vector2f(0, 1));
        return kickTo(Vector2f(half_field_length, 0));
        return kickRel();
        return goTo(Vector2f(-5, 1), -30, false);
        return shootRel();
        return goToAvoidBlocks(WM.getFlagRelPos2D(Vision::G2R), WM.getFlagPol2D(Vision::G2R).y(), true);

        return goToBallBack();
        mCameraMotionMode = 4; //============================================
        return kickRel();
        return shootRel();
        return goToBallBack();
        return dribbleToOppGoal();

        //=======================test
        const perception::JointPerception& jp = WM.lastPerception().joints();
        for (int i = 14; i >= 12; i--) {
            //printf("jid[%d]: %.2f",i,jp[i].angle());
        }

        //========================


        //const Vector3f& acc=WM.getMyAcc();
        //printf("%.2f %.2f %.2f\n",acc.x(),acc.y(),acc.z());

        return kickRel();

        //mCameraMotionMode=-2;
        /*
                printf("size: %d\t\t",mTask.getSubTaskListSize());
                shared_ptr<Task> curTask= mTask.getFirstSubTask();
                if(NULL!=curTask.get())
                {
                        shared_ptr<Walk> curWalk= shared_dynamic_cast<Walk>(curTask);
                        shared_ptr<WalkRel> curWalkRel= shared_dynamic_cast<WalkRel>(curTask);
                        shared_ptr<BasicKick> curKick= shared_dynamic_cast<BasicKick>(curTask);

                        if(NULL!=curWalk.get())
                                printf("first: walk");

                        if(NULL!=curWalkRel.get())
                                printf("first: walkRel");

                        if(NULL!=curKick.get())
                                printf("first: kick");
                }
                printf("\n");

         */
        //---------------------------------------------------------------------------------

        //return shared_ptr<Action>();
        //return goToRel(Vector2f(0,1),0);
        /////////////////////////////////////////////////////////////// TT test
        /*static int waitSign=0;
        waitSign++;
        cout<<waitSign<<endl;
        if(waitSign<100)
        {
                shared_ptr<Action> emptyAct;
                return emptyAct;
        }
        waitSign=0;*/

        //WM.setTurnCameraSign(true);

        /*
        static float x,y,ang;
        static bool isFirstTime=true;
        if(isFirstTime)
        {
                isFirstTime=false;
                ifstream inFile("tt.txt",ios::in);
                if(NULL==inFile){
                        printf("no input file\n");
                        return shared_ptr<Action>();
                }

                inFile>>x>>y>>ang;
                inFile.close();
        }

        return goToRel(Vector2f(x,y),ang);
         */

        /*
        shared_ptr<JointAction> adjustBodyAct(new JointAction);
        //adjustBodyAct->set4Search(i,v);
        adjustBodyAct->set(i1,v);
        adjustBodyAct->set(i2,v);

        perception::JointPerception jp=WM.lastPerception().joints();
        printf("jid=%d\t%.2f\tjid=%d\t%.2f\n",i1,jp[i1].angle(),i2,jp[i2].angle());
         */

        /*perception::JointPerception jp=WM.lastPerception().joints();
        for(int i=0;i<jp.jointMap().size();i++)
        {
                float deltaAngle=jp[i].angle()-niceJointAngle[i];
                //printf("jid=%d \t%f\n",i,deltaAngle);
                if( deltaAngle > 0.1f )
                {
                        needAdjust=true;
                        adjustBodyAct->set(i,-5);
                }
                else if( deltaAngle < -0.1f )
                {
                        needAdjust=true;
                        adjustBodyAct->set(i,5);
                }
                //printf("jid=%d \t%f\n",i,jp[i].angle());
        }*/

        //return adjustBodyAct;

        /*
        WM.setTurnCameraSign(false);

        static float niceJointAngle[22];
        static bool isFirstTime=true;
        if(isFirstTime)
        {
                isFirstTime=false;
                ifstream inFile("tt.txt",ios::in);
                if(NULL==inFile){
                        printf("no input file\n");
                        return shared_ptr<Action>();
                }

                for(int i=0;i<22;i++)
                        inFile>>niceJointAngle[i];
                inFile.close();
        }
         */

        /////////////////////////////////////////////////////////////// TT test
        /*static int waitSign=0;
        waitSign++;
        if(0==waitSign%10) cout<<waitSign<<endl;
        if(waitSign<100){
                shared_ptr<JointAction> emptyAct=shared_ptr<JointAction>(new JointAction);
                return emptyAct;
        }

        waitSign=0;*/

        /*const static float niceJointAngle[22]={0.00f,0.00f,
                -88.71f,
                0.00f,
                90.00f,
                15.64f,

                -91.29f,
                0.00f,
                -90.00f,
                -14.36f,

                0.01f,
                0.02f,
                35.31f,
                -60.09f,
                34.79f,
                -0.02f,

                0.00f,
                -0.02f,
                38.80f,
                -66.62f,
                37.83f,
                0.02f};*/

        /*const static float niceJointAngle[22]={0.00f,0.00f,
                0.00f,//-90.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,//-90.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f,
                0.00f};*/

        /*
        shared_ptr<JointAction> adjustBodyAct(new JointAction);		//it has been initialized
        perception::JointPerception jp=WM.lastPerception().joints();
        adjustingBody=false;
        for(int i=2;i<jp.jointMap().size();i++)//0 or 2
        {
                if(1==i)
                        continue;		//dont turn head in Y direction

                float deltaAngle=jp[i].angle()-niceJointAngle[i];
                //printf("jid=%d \t%f\n",i,deltaAngle);
                if( deltaAngle > 0.2f )
                {
                        adjustingBody=true;
                        adjustBodyAct->set(i,-5);
                        if(13==i||19==i) adjustBodyAct->set(i,-10);
                        //if(0==i||1==i) adjustBodyAct->set4Search(i,deltaAngle*(-20));
                }
                else if( deltaAngle < -0.2f )
                {
                        adjustingBody=true;
                        adjustBodyAct->set(i,5);
                        if(13==i||19==i) adjustBodyAct->set(i,10);
                        //if(0==i||1==i) adjustBodyAct->set4Search(i,deltaAngle*(-20));
                }
                //printf("jid=%d \t%f\n",i,jp[i].angle());
        }
        if(adjustingBody)
                return adjustBodyAct;
         */

        //============================================
        /*perception::JointPerception jp=WM.lastPerception().joints();
        for(int i=0;i<jp.jointMap().size();i++)
        {
                if( 0==i || 2==i || 6==i || 10==i || 16==i )
                        printf("\n");
                printf("%.2f\n",jp[i].angle());
        }*/
        //===========================================
    }
} // namespace soccer

