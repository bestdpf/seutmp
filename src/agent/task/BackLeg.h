
#ifndef TASK_BACK_LEG_H
#define TASK_BACK_LEG_H

//#define ENABLE_TASK_BACK_LEG_LOG


#include "Task.h"
#include "Kick.h"
#include "math/Math.hpp"
#include "perception/JointPerception.h"
#ifdef ENABLE_TASK_BACK_LEG_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{
	
	using namespace math;
    using namespace serversetting;
	using namespace boost;
	
    class BackLeg: public Task
    {
    public:
        BackLeg(Task* primary);
	
        virtual bool isDone() const;
	
		shared_ptr<action::Action> perform();

        //virtual bool revise( boost::shared_ptr<Task> rt );
        
    private:
		
        perception::JointPerception mDesiredJoints;
	
		Kick::PoseParameter mPoseParameter;
	
		shared_ptr<action::Action> mActionCache;
	
	DECLARE_STATIC_GRAPHIC_LOGGER;
    };
    
} //namespace task

#endif // TASK_BACK_LEG_H
