/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef TASK_BASIC_KICK_H
#define TASK_BASIC_KICK_H

#define ENABLE_TASK_BASIC_KICK_LOG

#include "Task.h"
#include "math/Math.hpp"
#include "perception/JointPerception.h"
#include "Kick.h"
#include "robot/humanoid/Humanoid.h"
#ifdef ENABLE_TASK_BASIC_KICK_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{

class BasicKick: public Task
{
public:
	BasicKick( const math::Vector2f& target, const math::Vector2f& movePos, const float biasAngle, bool isLeftLeg = false,Task* primary = NULL );

	virtual boost::shared_ptr<action::Action> perform();

	virtual KickParameter calKickParameter();

	virtual bool isAchieveable () const;

	virtual bool isDone() const;

	typedef std::map<math::AngDeg, float> TAnkleDist;

	typedef std::map<float, float> TDurationDist;


protected:

	virtual boost::shared_ptr<action::Action> amendAccJointPose(const float t);

	virtual boost::shared_ptr<action::Action> amendBaituiJointPose(const float t);

	virtual boost::shared_ptr<action::Action> amendBaituiJointPose2(const float t);

	/*
	* 生成踢球脚ankle角度到球运动距离的map
	*/
	virtual void generateAnkleDistTable() = 0;

	virtual void generateDurationDistTable() = 0;

	virtual AngDeg getDesiredAnkleAngle() = 0;

	virtual float getDesiredDuration() = 0;

	virtual bool shouldContinueKick();

protected:

	/** moving posiont when the ball is at the origin and target is on the axis x */
	const math::Vector2f mMovePos;

	static TAnkleDist mAnkleDist;

	static TDurationDist mDurationDist;

	math::Vector2f mTarget;

	const bool mIsLeftLeg;

	KickParameter mKickParameter;

	float mMoveCoMDuration;

	float mAmendDuration;

	std::map<unsigned int, math::AngDeg> mAccJointAngles;

	std::map<unsigned int, math::AngDeg> mBaituiJointAngles;

	float mAccStartTime;

	float mBaituiStartTime;

	float mBaituiDuration;

	Vector3f mErrPosBall;

	robot::humanoid::Humanoid::ESkeleton mKickFootSkeletion;

	std::string mRaiseRFootTask;
	std::string mRaiseLFootTask;

	std::string mBaiRtuiTask;
	std::string mBaiLtuiTask;

	std::string mAccRTask;
	std::string mAccLTask;

	std::string mBaituiTask;

	std::string mAccTask;

	const float mBiasAngle;

	boost::shared_ptr<action::Action> mAccAct;

	bool mShouldStop;

	float mSquatDuration;

	float mSwingFootHoldTime;

	math::Vector2f mBallPos2D;

	DECLARE_STATIC_GRAPHIC_LOGGER;
};


} //end of namespace task

#endif // TASK_BASIC_KICK_H

