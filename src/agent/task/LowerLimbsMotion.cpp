/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: LowerLimbsMotion.cpp 2748 2009-04-01 14:09:43Z zyj $
 *
 ****************************************************************************/

#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "controller/Timing.h"
#include "LowerLimbsMotion.h"
#include "controller/ArmMotion.h"

namespace task{
    using namespace std;
    using namespace boost;
    using namespace math;
    using namespace serversetting;
    using namespace perception;
    using namespace action;
    using namespace controller;
    
    LowerLimbsMotion::LowerLimbsMotion(float t, Task* primary)
        :Task(t,primary)
    {
    }

    bool LowerLimbsMotion::isDone() const
    {
        return isTimeOut();
    }

    boost::shared_ptr<action::Action> LowerLimbsMotion::perform()
    {
        Task::perform();
        mDesiredJoints = WM.predictedPerception().joints().jointAngles();
        calJoints(mDesiredJoints);
	/*
        const TransMatrixf& myOigMat = WM.getMyOriginTrans();
        TransMatrixf temp = myOigMat;
        temp.transfer(mDesiredBody);
        temp = myOigMat;
        temp.transfer(mDesiredFootL);
        temp = myOigMat;
        temp.transfer(mDesiredFootR);
        */
        float t = getRemainTime();

        shared_ptr<const LowerLimbsMotion> next =
            shared_dynamic_cast<const LowerLimbsMotion>( getNextOfAllSubTask() );
        if ( NULL != next.get() ){
            // the next task is LowerLimbsMotion too
            std::map<unsigned int, math::AngDeg> angles = mDesiredJoints;
            if ( next->calJoints(angles) )
            {
                LOG_PRINT("control","control with next task");
                return  Timing::control(WM.predictedPerception().joints(),
                                        mDesiredJoints, t,
                                        angles, next->getRemainTime());
            }
        }
        return controller::Timing::control(WM.predictedPerception().joints(),
                                           mDesiredJoints, t,
                                           mDesiredJoints, -1);
    }

    bool LowerLimbsMotion::revise( boost::shared_ptr<Task> /*rt*/ )
    {
        return false;
    }

    bool LowerLimbsMotion::calJoints(std::map<unsigned int, math::AngDeg>& angles) const
    {
        bool suc = true;
        if ( !HUMANOID.legInverseKinematics(true, mDesiredBody, mDesiredFootL, angles) ){
             cerr<<"Left Leg IK failed!"<<endl;
            suc = false;
        }
        if ( !HUMANOID.legInverseKinematics(false, mDesiredBody, mDesiredFootR, angles) ) {
            cerr<<"Right Leg IK failed!"<<endl;
            suc = false;
        }
        // controller::ArmMotion::control(mDesiredJoints);
        controller::ArmMotion::control(angles, mDesiredFootL.p(), mDesiredFootR.p(), mDesiredBody.p());
        return suc;
    }
};
