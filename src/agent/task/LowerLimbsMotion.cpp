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

    DEFINE_STATIC_GRAPHIC_LOGGER(LowerLimbsMotion)
    #define OPEN_LOWER_LIMBS_MOTION_LOGFILE \
	;
        /*BEGIN_ADD_STATIC_LOG_LAYER(LowerLimbsMotion) \
        ADD_LOG_LAYER("box"); \
        ADD_LOG_LAYER("pos"); \
        ADD_LOG_LAYER("rotation"); \
        ADD_LOG_LAYER("task");\
        ADD_LOG_LAYER("control");\
    END_ADD_STATIC_LOG_LAYER(LowerLimbsMotion)*/

    LowerLimbsMotion::LowerLimbsMotion(float t, Task* primary)
        :Task(t,primary)
    {
        OPEN_LOWER_LIMBS_MOTION_LOGFILE
    }

    LowerLimbsMotion::LowerLimbsMotion(const TransMatrixf& body,
                                       const TransMatrixf& footL,
                                       const TransMatrixf& footR,
                                       float t,
                                       Task* primary)
        :Task(t,primary),
         mDesiredBody(body),
         mDesiredFootL(footL),
         mDesiredFootR(footR)
    {
        OPEN_LOWER_LIMBS_MOTION_LOGFILE
    }

    bool LowerLimbsMotion::isDone() const
    {
        return isTimeOut();

        if ( isTimeOut() ) return true;

		//return false;																	/////terrymimi-note add it

        map<unsigned int, math::AngDeg> jc = WM.predictedPerception().joints().jointAngles();

        map<unsigned int, math::AngDeg>::const_iterator end = mDesiredJoints.end();
        for( map<unsigned int, math::AngDeg>::const_iterator iter=mDesiredJoints.begin();
             iter!=end; ++iter )
        {
            if ( fabs( calClipAng(
                          jc[iter->first],
                          iter->second ) ) > 1.0f ){
                return false;
            }
        }

        const static float errThrehold = 0.001;
        const TransMatrixf& myOig = WM.getMyOriginTrans();
        // body
        TransMatrixf err = myOig;
        err.transfer( mDesiredBody );
        err -= WM.getBoneTrans(robot::humanoid::Humanoid::TORSO);
        if ( err.squareLength() > errThrehold ) return false;

        // left foot
        err = myOig;
        err.transfer( mDesiredFootL );
        err -= WM.getBoneTrans(robot::humanoid::Humanoid::L_FOOT);
        if ( err.squareLength() > errThrehold ) return false;

        // right foot
        err = myOig;
        err.transfer( mDesiredFootR );
        err -= WM.getBoneTrans(robot::humanoid::Humanoid::R_FOOT);
        if ( err.squareLength() > errThrehold ) return false;

        return true;
    }

    boost::shared_ptr<action::Action> LowerLimbsMotion::perform()
    {
        Task::perform();
        mDesiredJoints = WM.predictedPerception().joints().jointAngles();
        calJoints(mDesiredJoints);

#ifdef ENABLE_LOG
#ifdef ENABLE_TASK_LOWER_LIMBS_MOTION_LOG
        LOG_PRINTF("task","this is %s", typeid(this).name());
        const TransMatrixf& myOigMat = WM.getMyOriginTrans();
        TransMatrixf temp = myOigMat;
        temp.transfer(mDesiredBody);
        LOG_RED_BOX("box",temp,HUMANOID.getBone(robot::humanoid::Humanoid::TORSO)->body()->size());
        LOG_PRINT_MATRIX_3X3("rotation",mDesiredBody.R());
        LOG_PRINT_VECTOR3("pos",mDesiredBody.p());

        temp = myOigMat;
        temp.transfer(mDesiredFootL);
        LOG_RED_BOX("box",temp,HUMANOID.getBone(robot::humanoid::Humanoid::L_FOOT)->body()->size());
        LOG_PRINT_MATRIX_3X3("rotation",mDesiredFootL.R());
        LOG_PRINT_VECTOR3("pos",mDesiredFootL.p());

        temp = myOigMat;
        temp.transfer(mDesiredFootR);
        LOG_RED_BOX("box",temp,HUMANOID.getBone(robot::humanoid::Humanoid::R_FOOT)->body()->size());
        LOG_PRINT_MATRIX_3X3("rotation",mDesiredFootR.R());
        LOG_PRINT_VECTOR3("pos",mDesiredFootR.p());
        LOG_FLUSH;
#endif // ENABLE_TASK_LOWER_LIMBS_MOTION_LOG
#endif // ENABLE_LOG

        float t = getRemainTime();

        // return controller::Timing::control(WM.predictedPerception(),
        //                                    mDesiredJoints, t);

        shared_ptr<const LowerLimbsMotion> next =
            shared_dynamic_cast<const LowerLimbsMotion>( getNextOfAllSubTask() );
        if ( NULL != next.get() ){
            // cout<<"control with next task"<<endl;
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

        // cout<<"control as last task"<<endl;
        LOG_PRINT("control","control as last task");
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
             //cerr<<"Left Leg IK failed!"<<endl;
            suc = false;
        }
        if ( !HUMANOID.legInverseKinematics(false, mDesiredBody, mDesiredFootR, angles) ) {
            //cerr<<"Right Leg IK failed!"<<endl;
            suc = false;
        }
        // controller::ArmMotion::control(mDesiredJoints);
        controller::ArmMotion::control(angles, mDesiredFootL.p(), mDesiredFootR.p(), mDesiredBody.p());
        return suc;
    }



    void LowerLimbsMotion::avoidFootCollide(TransMatrixf& moveFoot,
                                            const TransMatrixf& supFoot)
    {
        return;

        // consider x-y plan only
        Vector2f mfc, sfc;
        mfc = moveFoot.pos();
        sfc = supFoot.pos();
        AngDeg mfa = moveFoot.rotatedAngZ();
        AngDeg sfa = supFoot.rotatedAngZ();
        Vector2f size;
        const Vector3f& footSize = HUMANOID.getFootSize();
        size.x() = footSize.x();
        size.y() = footSize.y();
        ConvexPolygonf mcp, scp;
        mcp.createRectangle(mfc, mfa, size);
        scp.createRectangle(sfc, sfa, size);

        //vector<Vector2f> intersections;
        while ( mcp.contain(scp/*, intersections*/)
                || scp.contain(mcp/*, intersections*/) ){
            /*std::cout<<"intersect0 size"<<intersections.size()<<std::endl;
            for( vector<Vector2f>::const_iterator iter = intersections.begin();
                 iter!=intersections.end(); ++iter){
                std::cout<<*iter<<'\n';
                }*/
            Vector2f midify = mfc - sfc;
            midify.normalize();
            midify*=0.01;
            mfc += midify;
            mcp.createRectangle(mfc, mfa, size);
        }

        moveFoot.pos().x() = mfc.x();
        moveFoot.pos().y() = mfc.y();
    }

};
