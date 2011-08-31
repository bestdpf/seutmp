/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WalkRel.h 2011-03-07 TT $
 *
 ****************************************************************************/


#ifndef TASK_WALKREL_H
#define TASK_WALKREL_H


#include "Task.h"
#include "math/Math.hpp"

//#define ENABLE_TASK_WALKREL_LOG

#ifdef ENABLE_TASK_WALKREL_LOG
	#include "logger/Logger.h"
#else
	#include "logger/NoLogger.h"
#endif


namespace task{

class WalkRel: public Task
{
public:

	/**
	 * create a walk task by walking target and
	 * the direction while reach the target position
	 *
	 * @param target target position
	 * @param direction the direction while reach the target position
	 * @param primary the primary task which create this task
	 *
	 */
	WalkRel(const math::Vector2f& target,
			math::AngDeg direction=0,
			bool avoidBall=false,
			Task* primary=NULL);

	virtual bool isDone() const;

	virtual bool revise( boost::shared_ptr<const Task> rt );

	virtual void updateSubTaskList();

	static void setWalkHeight( float h )
	{
		mWalkHeight = h;
	}

	static float getWalkHeight()
	{
		return mWalkHeight;
	}

	///////////////////////////////// TT test
	void stopWalk()
	{
		mShouldStop=true;
		mTarget=math::Vector2f(0,0);
		mDirection=0;
	}

	float getStepLength()
	{
		return mPreSize.length();
	}



private:
	/// the target of the walk task
	math::Vector2f mTarget;

	/// the desired driection while reach the target
	math::AngDeg mDirection;

	math::Vector2f mPreSize;

	/// vaiables of path planning
	math::Vector2f mPlanTarget;
	math::AngDeg mPlanDir;

	/// the error threshold for isDone
	math::Vector2f mSizeErrorThreshold;
	math::AngDeg mDirErrorThreshold;

	static float mWalkHeight;

	float mIsWalking;

	/// should I avoid the ball
	bool mAvoidBall;

	//TT, for Walk and WalkRel
	bool mShouldStop;

	// this class handls logging
	DECLARE_STATIC_GRAPHIC_LOGGER;
}; //end of class WalkRel


} //end of namespace task

#endif //TASK_WALKREL_H

