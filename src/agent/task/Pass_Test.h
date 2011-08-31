/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: 
 *
 ****************************************************************************/

/**
 * @file   Pass_Test.h
 * @author Chen Si <cszzys@gmail.com>
 * @date   Mon Mar 31  2008
 * 
 * @brief  test ball pass
 * 
 */
 
#ifndef TASK_PASS_TEST_H
#define TASK_PASS_TEST_H

#define ENABLE_TASK_PASS_TEST_LOG

#include "Task.h"
#include "math/Math.hpp"
#include "perception/JointPerception.h"
#include <vector>
#include "core/WorldModel.h"
#include "SoccerDefines.h"
#ifdef ENABLE_TASK_PASS_TEST_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif



namespace task{
		 using namespace serversetting;
	 using namespace math;
         using namespace std;
	class PassTest: public Task
	{
	public:
		
		struct PoseParameter
		{
			TransMatrixf hip;
			TransMatrixf footL;
			TransMatrixf footR;
		};
		
		PassTest(Task* primary);
	
	private:
        /// target position of pass
        Vector2f mTargetPos;
	
		/// global position of the target contact point on the ball
		Vector3f mTargetContactPos;
	
		/// set of tested contact points
		vector<Vector3f> mTestContactPointSet;
	
		bool mIsLeftLeg;
	
		perception::JointPerception mDesiredPose;
	
		PoseParameter mPoseParameter;
	
		void generateTestContactPointSet(float samplingDistance);
	
		void writeTestPointsToFile();
	
	DECLARE_STATIC_GRAPHIC_LOGGER;
		
	};
	
}	//namespace task

#endif // TASK_PASS_TEST_H
