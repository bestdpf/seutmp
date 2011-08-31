/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/


#ifndef TASK_WALK_H
#define TASK_WALK_H

#define ENABLE_TASK_WALK_LOG

#include "Task.h"
#include "math/Math.hpp"
#ifdef ENABLE_TASK_WALK_LOG
	#include "logger/Logger.h"
#else
	#include "logger/NoLogger.h"
#endif


namespace task
{

using namespace math;


class Walk: public Task
{
public:
	/**
	 * create a walk task by walking target and
	 * the direction while reach the target position
	 *
	 * @param target target position
	 * @param direction the direction while reach the target position
	 * @param should avoid colliding the ball
	 * @param if distance and angle error is smaller than the threshold, I can stop
	 * @param zero indicates that the next task is NOT kicking; -1 and 1 indicate that the next task is Kicking and that the support foot is right and left foot respectively.
	 * @param primary the primary task which create this task
	 *
	 */
	Walk( const math::Vector2f& target,
		  math::AngDeg direction,
		  bool avoidBall, bool is4Kick = false,Vector3f threshold = Vector3f(0.01,0.01,3), int isForKick = 0, bool isFastWalking = false,bool yuandi = false,
		  Task* primary = NULL );

	//////terrymimi change (0.01,0.01.3) to ()

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

	const math::Vector2f& getTarget() const
	{
		return mTarget;
	}

	const math::AngDeg& getDirection() const
	{
		return mDirection;
	}

	static int mForDeadBall;

	/////////////////////////////// TT test
	void stopWalk()
	{
		mShouldStop=true;
	}

	float getStepLength()
	{
		return mPreSize.length();
	}


protected:
	/// class for path palnning
	class State
	{
		math::Vector2f mP;
		math::Vector2f mPreP;
		float mCosted;
		float mMoreCost;
	public:
		State(const math::Vector2f& p, float costed, float moreCost)
		:mP(p), mCosted(costed), mMoreCost(moreCost)
		{
		}

		const math::Vector2f& p() const
		{
			return mP;
		}

		float moreCost() const
		{
			return mMoreCost;
		}

		float cost() const
		{
			return mCosted + mMoreCost;
		}

		float costed() const
		{
			return mCosted;
		}

		State previous() const
		{
			return State(mPreP,0,0);
		}

		void setPrevious(const State& p)
		{
			mPreP = p.p();
		}

		bool sameAs(const State& r) const
		{
			return fabs(mP.x()-r.p().x()) < 0.01f
			&& fabs(mP.y()-r.p().y()) < 0.01f;
		}

		bool operator<(const State& r) const
		{
			return cost() < r.cost();
		}

	};

	/**
	 * plan the templated walking target position,
	 * the robot should avoid to collide with ball, wall and others
	 */
	void planPath();

	/**
	 * generate the possible state to go
	 *
	 * @param start the start state
	 * @param target the target state
	 * @param res return the possible state from start state
	 */
	void generatePossibleState( const State& start, const State& target,
								const State& final,
								std::vector<State>& res,
								std::vector<State>& unReach) const;

	/**
	 * this function setup the blocks in the field,
	 * the robot should avoid to collide these blocks while walking,
	 * currently, the blocks are segments
	 */
	void setupBlocks();

	/**
	 * build a block according to the relative position with the robot
	 * and the given radius
	 *
	 * @param p the postion of the block
	 * @param radius the radius of the block
	 *
	 * @return the block Segment2f
	 */
	math::Segment2f buildBlock(const math::Vector2f& p, float radius) const;

	std::vector<Segment2f> buildBlockForBall(const math::Vector2f& p, float radius) const;


	/**
		* This function is for the kick task, because origin should be
		* calculated according to the support foot expected by the kick
		* task if it is the next task.
	*/
	math::Vector2f calMyOrigin2D ();

protected:
	/// the target of the walk task
	math::Vector2f mTarget;

	/// the desired driection while reach the target
	math::AngDeg mDirection;

	math::Vector2f mPreSize;

	/// vaiables of path planning
	math::Vector2f mPlanTarget;
	math::AngDeg mPlanDir;

	/// blocks to avoid collding
	std::vector<math::Segment2f> mBlocks;

	/// the error threshold for isDone
	math::Vector2f mSizeErrorThreshold;
	math::AngDeg mDirErrorThreshold;

	static float mWalkHeight;

	bool mIsWalking;

	/// should I avoid the ball
	bool mAvoidBall;

	bool shouldback;

	/// the next task especially for kicking
	int mForKick;

	bool Yuandi;

	bool mIs4Kick;

	//TT, for Walk and WalkRel
	bool mShouldStop;

	// this class handles logging
	DECLARE_STATIC_GRAPHIC_LOGGER;
};


} //end of namespace task

#endif // TASK_WALK_H

