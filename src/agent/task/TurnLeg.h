
#ifndef TASK_TURN_LEG_H
#define TASK_TURN_LEG_H

//#define ENABLE_TASK_TURN_LEG_LOG


#include "Task.h"
#include "Kick.h"
#include "math/Math.hpp"
#include "perception/JointPerception.h"
#ifdef ENABLE_TASK_TURN_LEG_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{
	
	using namespace math;
    using namespace serversetting;
	using namespace boost;
	
    class TurnLeg: public Task
    {
    public:
        TurnLeg(Task* primary);
	
        virtual bool isDone() const;
	
		shared_ptr<action::Action> perform();

        //virtual bool revise( boost::shared_ptr<Task> rt );
        
    private:
		
        perception::JointPerception mDesiredJoints;
	
		Kick::PoseParameter mPoseParameter;
	
		shared_ptr<Action> mActionCache;
	
	DECLARE_STATIC_GRAPHIC_LOGGER;
    };
    
} //namespace task

#endif // TASK_TURN_LEG_H
