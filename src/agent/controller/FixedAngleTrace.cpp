/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "configuration/Configuration.h"
#include "core/WorldModel.h"
#include "FixedAngleTrace.h"
#include <fstream>
#include <sstream>

namespace controller {

using namespace std;
using namespace boost;
using namespace math;
using namespace perception;

FixedAngleTrace::FixedAngleTrace()
{
    init();

	/*BEGIN_ADD_LOG_LAYER(FixedAngleTrace)
	ADD_LOG_LAYER("task");
	//ADD_LOG_LAYER("WalkOnline");
	END_ADD_LOG_LAYER(FixedAngleTrace);*/
}

#define TASK_FILE "data/nao.txt"
#define POSE_FILE "data/nao_pose.txt"

void FixedAngleTrace::init()
{
	mTaskMap.clear();
	mPoseMap.clear();
	loadConfigFile<Task>(TASK_FILE,mTaskMap);
	loadConfigFile<perception::JointPerception>(POSE_FILE,mPoseMap);

	/*for( int i=0; i<10; i++ )
	  {
	  stringstream ss;
	  ss<<"data/walk_pose_0."<<i<<".txt";
	  loadConfigFile<perception::JointPerception>(ss.str(),temp);
	  mPoseMapVec.push_back(temp);
	  }*/

	// we should give an initial value here!!
	mTask = &(mTaskMap["squat"]);
	mIsFinished = true;
	mIsStartNextTask = false;
}

template<typename DATATYPE>
bool FixedAngleTrace::loadConfigFile(const string& filename, map<string, DATATYPE>& map)
{
	ifstream ifs;
	ifs.open(filename.c_str());
	if ( !ifs )
	{
		cerr<<"FixedAngleTrace can not open file: "<<filename<<endl;
		return false;
	}

	while( !ifs.eof() )
	{
		string nameLine, dataLine;
		getline(ifs,nameLine);
		if ( nameLine.empty() ) continue; // skip empty line
		if ( '#' == nameLine[0] ) continue; // skip comment line
		if ( ifs.eof() )
		{
			cerr<<"FixedAngleTrace data breaks: "<<nameLine<<endl;
			return false;
		}
		stringstream ss;
		do
		{
			dataLine.clear();
			getline(ifs,dataLine);
			if ( '#' != dataLine[0] )
			{
				ss<<dataLine<<' ';
			}
		}
		while ( !dataLine.empty() );

		//cout<<ss.str()<<endl;
		ss>>map[nameLine];
	}
	return true;
}

shared_ptr<action::Action> FixedAngleTrace::control(const string& taskName)
{
	float simTime = WM.getSimTime();
	LOG_PRINTF("task","current time: %f", simTime);

	if( taskName!=mTaskName && true==mTask->changeable )
	{
		mEntryTaskName=taskName;
		setCurrentTask(taskName);
		LOG_PRINT("task"," start new task "+mTaskName);
		LOG_PRINT("task","current pos is "+getCurrentPose());
		LOG_FLUSH;

		//return Timing::control(WM.predictedPerception(), mDesiredPose, mTask->time);
		return Timing::control(WM.lastPerception(), mDesiredPose, mTask->time); //TT 1
	}

	float remainTime = mTaskBeginTime + mTask->time - simTime;
	if ( remainTime > 0.005f )
	{
		LOG_PRINT("task"," continue task "+mTaskName);
		LOG_PRINT("task","current pos is "+getCurrentPose());
		LOG_FLUSH;
		shared_ptr<action::Action> act
			//= Timing::control(WM.predictedPerception(), mDesiredPose, remainTime);
			= Timing::control(WM.lastPerception(), mDesiredPose, remainTime); //TT 2
		int adjustFoot = mTask->adjustFoot;
		if ( mFootExchanged ) adjustFoot = -adjustFoot;
		// FOOT_ADJUSTER.adjust( adjustFoot, shared_static_cast<action::JointAction>(act) );
		mIsFinished = false;
		return act;
	}

	// the task should be finished
	//// if no next task
	if ( "null" == mTask->next )
	{
		//cout<<" stop at task "<<mTaskName<<' '<<mTask->next<<'\n';
		mIsFinished = true;
		mTaskName = "null";
		//mEntryTaskName = "null";
		LOG_PRINTF("task","do null");
		LOG_FLUSH;

		//return Timing::control(WM.predictedPerception(),mDesiredPose,(20.0f * serversetting::sim_step));
		return Timing::control(WM.lastPerception(),mDesiredPose,(20.0f * serversetting::sim_step)); //TT 3
	}

	//// start next task
	setCurrentTask( mTask->next );
	mIsStartNextTask = true;
	LOG_PRINT("task"," start next task "+mTaskName);
	LOG_FLUSH;
	//return Timing::control(WM.predictedPerception(), mDesiredPose, mTask->time);
	return Timing::control(WM.lastPerception(), mDesiredPose, mTask->time); //TT 4
}

shared_ptr<action::Action> FixedAngleTrace::controlPreferThan(const string& task, const string& oldTask)
{
    if ( "null" == mTaskName
         || ( task != mEntryTaskName && // do not give up the same task
         ( oldTask == "*"  // give up any old task
         || oldTask == mEntryTaskName // give up the given task
             )) )
    {
        mEntryTaskName=task; //don't update here
        setCurrentTask(task); //wait the update in control()
		LOG_PRINT("task"," give up old task, start new task "+mTaskName);
		//LOG_FLUSH; //should be comment, otherwise the forward words wouldn't be displayed
    }
    return control(task);
}

void FixedAngleTrace::setCurrentTask( const string& taskName )
{
    mTaskName = taskName;
    mTaskBeginTime = WM.getSimTime();
    mTask = &(mTaskMap[mTaskName]);
    mDesiredPose = mPoseMap[mTask->pose];
    mFootExchanged = mTask->changeFoot;
    mIsFinished = false;
    if(mFootExchanged)
		mDesiredPose.exchange();
}

shared_ptr<action::Action> FixedAngleTrace::continueTask()
{
    mIsStartNextTask = false;
    return control(mTaskName);
}

float FixedAngleTrace::calTaskTime(const string& task) const
{
	string str = task;
	float time = 0;
	while ( "null" != str ){
		TTaskMap::const_iterator iter = mTaskMap.find(str);
		if ( mTaskMap.end() == iter ) break;
		time += iter->second.time;
		str = iter->second.next;
	}
	return time;
}

bool FixedAngleTrace::isPoseReached(const string& pose, AngDeg angErr ) const
{
	TPoseMap::const_iterator iterPose = mPoseMap.find(pose);
	if ( mPoseMap.end() == iterPose ){
		return true;
	}

	const JointPerception& cJoint = WM.lastPerception().joints();
	const JointPerception& dJoint = iterPose->second;
	const JointPerception::TJointMap& dJointMap = dJoint.jointMap();
	FOR_EACH(iter, dJointMap){
		AngDeg cAng = cJoint.jointAng(iter->first);
		AngDeg dAng = iter->second.angle();
		if ( abs(calClipAng(cAng,dAng)) > angErr ){
			return false;
		}
	}
	return true;
}

bool FixedAngleTrace::addTask(const string& name, const Task& t)
{
    bool add = (mTaskMap.find(name) == mTaskMap.end());
    mTaskMap[name] = t;
    return add;
}

bool FixedAngleTrace::addPose(const string& name, const perception::JointPerception& jp)
{
    bool add = (mPoseMap.find(name) == mPoseMap.end());
    mPoseMap[name] = jp;
    return add;
}


void FixedAngleTrace::saveToFile()
{
  	ofstream oft(TASK_FILE);
    cout<<"save "<<mTaskMap.size()<<endl;

    FOR_EACH(iter,mTaskMap){
        oft<<iter->first<<'\n'
           <<iter->second<<'\n'
           <<endl;
    }

    ofstream ofp(POSE_FILE);
    FOR_EACH(iter,mPoseMap){
        ofp<<iter->first<<'\n'
           <<iter->second<<'\n'
           <<endl;
    }
}

bool FixedAngleTrace::changeTask(string taskName, Task task)
{
	if(mTaskMap.find(taskName) == mTaskMap.end())
			return false;
	else
	{
		task.changeFoot = mTaskMap[taskName].changeFoot;
		mTaskMap[taskName] = task;
		return true;
	}
}

std::string FixedAngleTrace::getCurrentPose()
{
	return mTask->pose;
}


std::ostream& operator<<(std::ostream &stream, FixedAngleTrace::Task& task)
{
    stream
        << task.pose << ' '
        << task.time << ' '
        << task.changeable << ' '
        << task.adjustFoot << ' '
        << task.next << ' '
        << task.changeFoot;
    return stream;
}

std::istream& operator>>(std::istream &stream, FixedAngleTrace::Task& task)
{
	stream
	>> task.pose
	>> task.time
	>> task.changeable
	>> task.adjustFoot
	>> task.next
	>> task.changeFoot;
	return stream;
}


} //end of namespace controller
