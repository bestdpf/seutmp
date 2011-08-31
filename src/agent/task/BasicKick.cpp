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
//mMovePos = movePos;

	/*Vector3f com;
	if(mIsLeftLeg)
	{
		mKickFootSkeletion = HUMANOID.L_FOOT;
		com = Vector3f( 0.7*HUMANOID.getHalfFeetWidth(),0,Walk::getWalkHeight () );
	}
	else
	{
		mKickFootSkeletion = HUMANOID.R_FOOT;
		com = Vector3f( (-0.7) * HUMANOID.getHalfFeetWidth(),0,Walk::getWalkHeight () );
	}*/

// add MoveCoM
	//TT test remove
	//shared_ptr<MoveCoM> mcom1( new MoveCoM (Vector3f(0,0,Walk::getWalkHeight ()), mMoveCoMDuration, this ));
	//mSubTaskList.push_back(mcom1);
	//shared_ptr<MoveCoM> mcom2( new MoveCoM (com, mMoveCoMDuration, this ) );
	//mSubTaskList.push_back(mcom2);

	/*BEGIN_ADD_STATIC_LOG_LAYER(BasicKick);
	ADD_LOG_LAYER("desiredDuration");
	ADD_LOG_LAYER("KickParameter");
	ADD_LOG_LAYER("amendJointPose");
	ADD_LOG_LAYER("amendBaituiJointPose");
	ADD_LOG_LAYER("isDone");
	ADD_LOG_LAYER("camera")
	END_ADD_STATIC_LOG_LAYER(BasicKick);*/
}

shared_ptr<action::Action> BasicKick::amendAccJointPose (const float t)
{
// angle map
	string accPoseName = FAT.taskMap().find(mAccTask)->second.pose;
	JointPerception jp = FAT.poseMap().find(accPoseName)->second;
	if(mIsLeftLeg)
		jp.exchange();
	map<unsigned int, AngDeg> jangles = jp.jointAngles();

// idx
	std::list<shared_ptr<const Bone> > idx;
	HUMANOID.findRoute( HUMANOID.TORSO, mKickFootSkeletion, idx );

// base matrix
	TransMatrixf bm = WM.getMyOriginTrans();
	bm.inverseTransfer(WM.getBoneTrans(HUMANOID.TORSO));

	map<unsigned int, TransMatrixf> mats;
	HUMANOID.forwardKinematics( idx, bm, jangles, mats );

// idealBallTMatOrigin
	TransMatrixf idealBallTMatGlobal;
	idealBallTMatGlobal.identity();
	idealBallTMatGlobal.pos() = Vector3f(0,0,serversetting::ball_radius);
	TransMatrixf idealOriginTMatGlobal;
	idealOriginTMatGlobal.rotationZ(-90.0);

	Vector2f temp2;
	if(mIsLeftLeg)
		temp2 = Vector2f( mMovePos.x(), (-1)*mMovePos.y() );
	else
		temp2 = mMovePos;

	idealOriginTMatGlobal.pos() = Vector3f( temp2.x(), temp2.y(), 0);
	LOG_AXES("amendJointPose", idealOriginTMatGlobal, 0.2);
	LOG_AXES("amendJointPose", WM.getMyOriginTrans(), 0.3);
	TransMatrixf idealBallTMatOrigin = idealOriginTMatGlobal;
	idealBallTMatOrigin.inverseTransfer(idealBallTMatGlobal);

// actualBallTMatOrigin
	TransMatrixf actualBallTMatGlobal;
	actualBallTMatGlobal.identity();
	actualBallTMatGlobal.pos() = WM.getBallGlobalPos();
	TransMatrixf actualBallTMatOrigin = WM.getMyOriginTrans();
	actualBallTMatOrigin.inverseTransfer(actualBallTMatGlobal);

// err of pos of ball
	mErrPosBall = idealBallTMatOrigin.pos() - actualBallTMatOrigin.pos();
	mErrPosBall.z() = 0;
	TransMatrixf amendFootTMatOrigin = mats[HUMANOID.getBoneId(mKickFootSkeletion)];
	amendFootTMatOrigin.pos() -= mErrPosBall;
//amendFootTMatOrigin.rotateLocalX(-10);
	LOG_PRINT_VECTOR3("amendJointPose",mErrPosBall);

	map<unsigned int, math::AngDeg> angles = jangles;
	if(mAccJointAngles.empty())
	{
		if(!HUMANOID.legInverseKinematics(mIsLeftLeg, bm, amendFootTMatOrigin, angles))
		{
			LOG_PRINT("amendJointPose", "legInverseKinematics failed");
		}
		else
		{
			LOG_PRINT("amendJointPose", "legInverseKinematics successful");
			jangles = angles;
			mAccJointAngles = jangles;
		}
	}
	else
	{
		LOG_PRINT("amendJointPose", "mAccJointAngles is NOT empty!");
		jangles = mAccJointAngles;
	}


#ifdef ENABLE_TASK_BASIC_KICK_LOG
	LOG_PRINTF("amendJointPose", "amendDuration:%f", t);
// origin pos
	Vector2f movePos = mKickParameter.movePos;
// ball pos
	const TransMatrixf& myOigMat = WM.getMyOriginTrans();
	TransMatrixf temp = myOigMat;
	temp.transfer(actualBallTMatOrigin);
	LOG_RED_SPHERE("amendJointPose", temp.pos(), serversetting::ball_radius);
	temp = idealOriginTMatGlobal;
	temp.transfer(idealBallTMatOrigin);
	LOG_BLUE_SPHERE("amendJointPose", temp.pos(), serversetting::ball_radius);
// foot TransMatrixf
	temp = myOigMat;
	temp.transfer(mats[HUMANOID.getBoneId(mKickFootSkeletion)]);
	shared_ptr<const Bone> rfoot = HUMANOID.getBone(HUMANOID.getBoneId(mKickFootSkeletion));
	LOG_BLUE_BOX("amendJointPose", temp, rfoot->collider()->size() );
	temp = myOigMat;
	temp.transfer(amendFootTMatOrigin);
	LOG_RED_BOX("amendJointPose", temp, rfoot->collider()->size() );
#endif

	LOG_FLUSH;

// keep foot horizontal
	const JointPerception& pPerception = WM.predictedPerception().joints();
	map<unsigned int, AngDeg> pAngles = pPerception.jointAngles();
	const TransMatrixf& suportFoot = mIsLeftLeg?WM.getRightFootTrans():WM.getLeftFootTrans();
	map<unsigned int, TransMatrixf> pMats;

	HUMANOID.forwardKinematics(mIsLeftLeg?HUMANOID.R_FOOT:HUMANOID.L_FOOT,
							   suportFoot, pAngles, pMats);

	unsigned int shank = HUMANOID.getBoneId(mIsLeftLeg?HUMANOID.L_SHANK:HUMANOID.R_SHANK);
	unsigned int foot = (mIsLeftLeg?20:14);
	const TransMatrixf& shankT = pMats[shank];
	AngDeg x = shankT.rotatedAngX();
	jangles[foot] -= x;

//if( t > 0 )
	return Timing::control(WM.lastPerception(), jangles, t);
//else
//return shared_ptr<action::Action>();
}

boost::shared_ptr<action::Action> BasicKick::amendBaituiJointPose(const float t)
{
	if(mBaituiJointAngles.empty())
	{
		amendAccJointPose (-1);

		if(!mAccJointAngles.empty())
		{
			string baituiPoseName = FAT.taskMap().find(mBaituiTask)->second.pose;
			string accPoseName = FAT.taskMap().find(mAccTask)->second.pose;
			JointPerception jpBaitui = FAT.poseMap().find(baituiPoseName)->second;
			JointPerception jpAcc = FAT.poseMap().find(accPoseName)->second;
			if(mIsLeftLeg)
			{
				jpBaitui.exchange();
				jpAcc.exchange();
			}
			map<unsigned int, AngDeg> baituiAngles = jpBaitui.jointAngles();
			map<unsigned int, AngDeg> accAngles = jpAcc.jointAngles();

			for(map<unsigned int, AngDeg>::const_iterator iter = baituiAngles.begin();
			   iter != baituiAngles.end();
			   iter ++)
			{
				unsigned int id = iter->first;
				mBaituiJointAngles[id] = mAccJointAngles[id] - accAngles[id] + baituiAngles[id];
			}

#ifdef ENABLE_LOG
#ifdef ENABLE_TASK_BASIC_KICK_LOG
// idx
			std::list<shared_ptr<const Bone> > idx;
			HUMANOID.findRoute( HUMANOID.TORSO, mKickFootSkeletion, idx );
// base matrix
			TransMatrixf bm = WM.getBoneTrans(HUMANOID.TORSO);

			map<unsigned int, TransMatrixf> mats;
			HUMANOID.forwardKinematics( idx, bm, mBaituiJointAngles, mats );
			shared_ptr<const Bone> rfoot = HUMANOID.getBone(HUMANOID.getBoneId(mKickFootSkeletion));
// amend foot pos
			LOG_RED_BOX("amendBaituiJointPose", mats[HUMANOID.getBoneId(mKickFootSkeletion)], rfoot->collider()->size());
// original foot pos
			map<unsigned int, AngDeg> temp = baituiAngles;
			HUMANOID.forwardKinematics( idx, bm, temp, mats );
			LOG_BLUE_BOX("amendBaituiJointPose", mats[HUMANOID.getBoneId(mKickFootSkeletion)], rfoot->collider()->size());
#endif
#endif

			LOG_FLUSH;
		}
		else
		{
			return FAT.controlPreferThan("oblique45III", "*");
		}
	}

	return Timing::control (WM.lastPerception(), mBaituiJointAngles, t);
}

shared_ptr<action::Action> BasicKick::amendBaituiJointPose2( const float t )
{
	string baituiPoseName = FAT.taskMap().find(mBaituiTask)->second.pose;
	JointPerception jpBaitui = FAT.poseMap().find(baituiPoseName)->second;
	if(mIsLeftLeg)
		jpBaitui.exchange();
	map<unsigned int, AngDeg> jangles = jpBaitui.jointAngles();
	map<unsigned int, AngDeg> temp = jangles;
	if(mBaituiJointAngles.empty())
	{
		amendAccJointPose (-1);
// idx
		std::list<shared_ptr<const Bone> > idx;
		HUMANOID.findRoute( HUMANOID.TORSO, mKickFootSkeletion, idx );
// base matrix
		TransMatrixf bm = WM.getBoneTrans(HUMANOID.TORSO);
		map<unsigned int, TransMatrixf> mats;

		HUMANOID.forwardKinematics( idx, bm, temp, mats );
		TransMatrixf kickFoot = mats[HUMANOID.getBoneId(mKickFootSkeletion)];
		kickFoot.pos() -= mErrPosBall;

#ifdef ENABLE_LOG
#ifdef ENABLE_TASK_BASIC_KICK_LOG
		shared_ptr<const Bone> rfoot = HUMANOID.getBone(HUMANOID.getBoneId(mKickFootSkeletion));
		LOG_RED_BOX("amendBaituiJointPose", mats[HUMANOID.getBoneId(mKickFootSkeletion)], rfoot->collider()->size());
		LOG_BLUE_BOX("amendBaituiJointPose", kickFoot, rfoot->collider()->size());
#endif
#endif

		if(!HUMANOID.legInverseKinematics(mIsLeftLeg, bm, kickFoot, temp))
		{
			LOG_PRINT("amendBaituiJointPose", "legInverseKinematics failed");
		}
		else
		{
			LOG_PRINT("amendBaituiJointPose", "legInverseKinematics successful");
			jangles = temp;
			mBaituiJointAngles = temp;
		}
	}
	else
	{
		jangles = mBaituiJointAngles;
	}

	LOG_FLUSH;
	return Timing::control(WM.lastPerception(), jangles, t);
}

shared_ptr<Action> BasicKick::perform()
{
	LOG_PRINTF("desiredDuration", "mSubTaskList.size=%d", mSubTaskList.size());
	LOG_FLUSH;

	if(!shouldContinueKick ())
	{
		mDuration = 0.0f;
		mSubTaskList.clear();
		return FAT.controlPreferThan("squat", "*");
	}

	shared_ptr<Action> act;
	if(mSubTaskList.size() > 0)
	{
		if(-1 == mStartTime)
		{
//FAT.setCurrentTask(mRaiseRFootTask);
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
		else	// start raise foot
		{
			if(!mIsLeftLeg)
				FAT.setCurrentTask(mRaiseRFootTask);
			else
				FAT.setCurrentTask(mRaiseLFootTask);
		}
	}

	string currentTaskName = FAT.currentTaskName();
	LOG_PRINTF("desiredDuration", "currentTaskName:%s", currentTaskName.c_str());

	if(currentTaskName == mRaiseRFootTask || currentTaskName == mRaiseLFootTask)
	{
		LOG_FLUSH;
		if(mIsLeftLeg)
		{
			return FAT.controlPreferThan(mRaiseLFootTask, "*");
		}
		else
		{
			return FAT.controlPreferThan(mRaiseRFootTask, "*");
		}
	}
	else if(currentTaskName == mBaituiTask)
	{
/* 			 if( mBaituiStartTime < 0 )
* 			 {
* 				 mBaituiStartTime = WM.getSimTime();
* 				 mBaituiDuration = FAT.taskMap().find(mBaituiTask)->second.time;
* 			 }
*
* 			 float baituiRemainingTime = mBaituiDuration + mBaituiStartTime - WM.getSimTime();
* 			 if( baituiRemainingTime <= 0.005 )
* 			 {
* 				 FAT.setCurrentTask(mAccTask);
* 				 //mAccJointAngles.clear();
* 			 }
*
* 			 return amendBaituiJointPose2 (baituiRemainingTime);
*/

		if(mBaituiStartTime < 0)
		{
			mBaituiStartTime = WM.getSimTime();
			mBaituiDuration = FAT.taskMap().find(mBaituiTask)->second.time;
			mDuration += mSwingFootHoldTime;
		}

		float baituiRemainingTime = mBaituiDuration + mBaituiStartTime - WM.getSimTime();

		if((baituiRemainingTime+mSwingFootHoldTime) <= 0.005)
		{
			FAT.setCurrentTask(mAccTask);
			return shared_ptr<JointAction>( new JointAction() );
		}
		else if(baituiRemainingTime >= 0.0f)
		{
			return amendBaituiJointPose2 (baituiRemainingTime);
		}
		else
		{
			return amendBaituiJointPose2 (baituiRemainingTime+mSwingFootHoldTime);
		}
	}
	else if(currentTaskName == mAccTask)
	{
		if(mAccStartTime < 0)
		{
			mAccStartTime = WM.getSimTime();
			mStartTime = mAccStartTime;
		}

		if(mAmendDuration < 0)
		{
			mAmendDuration = getDesiredDuration ();
//mDuration = mAmendDuration;
			mDuration = mAmendDuration + mSquatDuration;
		}

		float accRemainingTime = mAmendDuration + mAccStartTime - WM.getSimTime();
		float shootRemainingTime = getRemainTime ();
		LOG_PRINTF("desiredDuration", "accRemainingTime=%f,shootRemainingTime=%f", accRemainingTime, getRemainTime ());
/* 			 if( accRemainingTime < 0.005 )
* 				 FAT.setCurrentTask("squat");
*
* 			 return amendAccJointPose (mAmendDuration + mAccStartTime - WM.getSimTime());
*/

		if(shootRemainingTime >= (mSquatDuration + 0.005))
		{
			LOG_PRINT("desiredDuration","acc 1");
			LOG_FLUSH;
			return amendAccJointPose (mAmendDuration + mAccStartTime - WM.getSimTime());
		}
		else
		{
			LOG_PRINT("desiredDuration","acc accomplished");
			LOG_FLUSH;
			FAT.setCurrentTask("squat");
//return shared_ptr<action::JointAction>(new action::JointAction ());
			return FAT.controlPreferThan("squat", "*");
		}
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

KickParameter BasicKick::calKickParameter ()		// cal moving position and facing direction
{
	const Vector2f& ballPos2D = WM.getInterceptBallGlobalPos2D();		 //camera cancel it
//const Vector2f& ballPos2D = WM.getBallGlobalPos2D();					//camera add it
	AngDeg alpha;
	if(mBiasAngle <= 90.001f)
	{
		alpha = 90.0 - mBiasAngle - atan2Deg (mMovePos.x(), mMovePos.y());
//alpha = atan2Deg (mMovePos.x(), mMovePos.y()) - mBiasAngle;
	}
	else
	{
		alpha = atan2Deg (mMovePos.y(), mMovePos.x());
	}
	AngDeg theta;
	AngDeg beta;
	AngDeg gamma = calClipAng (mTarget - ballPos2D, Vector2f(1,0));
	if(mIsLeftLeg)
	{
		theta = gamma + mBiasAngle;
		beta = gamma - alpha;
	}
	else
	{
		theta = gamma - mBiasAngle;
		beta = gamma + alpha;
	}
	mKickParameter.movePos = pol2xyz(Vector2f(mMovePos.length(), beta )) + ballPos2D;
	mKickParameter.bodyAng = theta;

#ifdef ENABLE_TASK_BASIC_KICK_LOG
	LOG_GREEN_SPHERE("KickParameter",Vector3f(ballPos2D.x(),ballPos2D.y(),0),0.05);
	LOG_BLUE_SPHERE("KickParameter", Vector3f(mKickParameter.movePos.x(),mKickParameter.movePos.y(), 0.2), 0.02);
	LOG_PRINTF("KickParameter", "alpha=%f,theta=%f,beta=%f,gamma=%f", alpha, theta, beta, gamma);
	Vector2f myPos2D = WM.getMyGlobalPos2D();
	LOG_BLUE_LINE_2D("KickParameter", myPos2D, (myPos2D + Vector2f(2*cosDeg (theta), 2*sinDeg(theta))));
#endif

	LOG_FLUSH;

	return mKickParameter;
}

bool BasicKick::shouldContinueKick()
{
//return true;														//camera add it
	if(mBallPos2D.length() > 999.0f)
	{
		mBallPos2D = WM.getBallGlobalPos2D();
		return true;
	}
	else
	{
//return ((mBallPos2D-WM.getBallGlobalPos2D()).length() <= 0.05f);		//camera cancel it
//camera add it
		if((mBallPos2D-WM.getBallGlobalPos2D()).length() > 0.05f&&(FAT.currentTaskName() != "hu_kick2R"&&FAT.currentTaskName() != "hu_kick3L"&&FAT.currentTaskName() != "hu_kick2L"&&FAT.currentTaskName() != "hu_kick3R"))
			return false;
	}
	return true;														//camera add it
}


} //end of namespace task

