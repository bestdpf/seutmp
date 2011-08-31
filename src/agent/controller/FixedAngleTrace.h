/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef CONTROLLER_FIXED_ANGLE_TRACE_H
#define CONTROLLER_FIXED_ANGLE_TRACE_H

//#define ENABLE_FIXED_ANGLE_TRACE_LOG

#include "Singleton.hpp"
#include "../perception/Perception.h"
#include "Proportion.h"
#include "Timing.h"
#include "FootAdjuster.h"
#ifdef ENABLE_FIXED_ANGLE_TRACE_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace controller {

using namespace std;
using namespace boost;
using namespace math;
using namespace perception;

class FixedAngleTrace: public Singleton<FixedAngleTrace>
{
public:
	FixedAngleTrace();

    /**
     * load configuration from file
     */
    void init();

	/** an angle step means to desired angle and needed time */
	struct Task
	{
        std::string pose; // desired pose
		float time; // move in given time
		bool changeable; // can be changed by other tasks?
		int adjustFoot; // can adjust foot? which?
        std::string next; // next task name
		bool changeFoot; // will the foot changed in next task?
	};
	typedef std::map<std::string, Task> TTaskMap;
	typedef std::map<std::string, perception::JointPerception> TPoseMap;

    boost::shared_ptr<action::Action> control(const std::string& task);

    boost::shared_ptr<action::Action>
    controlPreferThan(const std::string& task, const std::string& oldTask);

    bool isFinished() const
	{
		return mIsFinished;
	}


    boost::shared_ptr<action::Action> continueTask();

    const TTaskMap& taskMap() const
	{
		return mTaskMap;
	}

    TTaskMap& taskMap()
	{
		return mTaskMap;
	}

    const TPoseMap& poseMap() const
	{
		return mPoseMap;
	}

    TPoseMap& poseMap()
	{
		return mPoseMap;
	}

    const Task& currentTask() const
	{
		return *mTask;
	}

    Task& currentTask()
	{
		return *mTask;
	}

    const perception::JointPerception* desiredPose() const
	{
		return &(mPoseMap.find(mTask->pose)->second);
	}

    perception::JointPerception* desiredPose()
	{
		return &mPoseMap[mTask->pose];
	}

    const std::string& currentTaskName() const
	{
		return mTaskName;
	}

    void setCurrentTask( const std::string& taskName);

    bool isStartNextTask() const
	{
		return mIsStartNextTask;
	}

    void setIsStartNextTask(bool b)
	{
		mIsStartNextTask = b;
	}

	float calTaskTime(const std::string& task) const;

	bool isPoseReached ( const std::string& pose, math::AngDeg angErr ) const;

    /**
     * add a task
     *
     * @param name string
     * @param t input task
     * @return true: add a new task, false: modify a task
     */
    bool addTask(const std::string& name, const Task& t);

    bool addPose(const std::string& name, const perception::JointPerception& jp);

    void saveToFile();

	bool changeTask(string taskName, Task task);

	std::string getCurrentPose();


protected:
	template<typename DATATYPE>
	bool loadConfigFile(const std::string& filename, std::map<std::string, DATATYPE>& map);


private:
    //Proportion mPctr;

	/** fixed angles */
	TTaskMap mTaskMap;
    TPoseMap mPoseMap;

	// current task
    std::string mEntryTaskName;
    std::string mTaskName;
	float mTaskBeginTime;
	Task* mTask;

	perception::JointPerception mDesiredPose;
	bool mFootExchanged; // moving foot

    // indicates if the task is finished,
    // if yes then we can do some other things
    bool mIsFinished;

    bool mIsStartNextTask;

	DECLARE_GRAPHIC_LOGGER;
};


#define FAT controller::FixedAngleTrace::GetSingleton()

std::ostream& operator<<(std::ostream &stream, FixedAngleTrace::Task& task);
std::istream& operator>>(std::istream &stream, FixedAngleTrace::Task& task);

} //end of namespace controller

#endif // CONTROLLER_FIXED_ANGLE_TRACE_H

