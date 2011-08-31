/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

/**
 * @file   LowerLimbsMotion.h
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Mon Oct  8 10:31:07 2007
 * 
 * @brief  this class handles lower limbs motion by given desired rotation
 * and position of body, left foot and right foot. It uses the robot inverse
 * kinematics.
 */


#ifndef TASK_LOWER_LIMBS_MOTION_H
#define TASK_LOWER_LIMBS_MOTION_H

#define ENABLE_TASK_LOWER_LIMBS_MOTION_LOG


#include "Task.h"
#include "math/Math.hpp"
#include "perception/JointPerception.h"
#ifdef ENABLE_TASK_LOWER_LIMBS_MOTION_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{

    class LowerLimbsMotion: public Task
    {
    public:
        /** 
         * create the motion of lower limbs
         * */
        LowerLimbsMotion(const math::TransMatrixf& body,
                         const math::TransMatrixf& footL,
                         const math::TransMatrixf& footR,
                         float t,
                         Task* primary);

        virtual bool isDone() const;

        virtual boost::shared_ptr<action::Action> perform();

        virtual bool revise( boost::shared_ptr<Task> rt );

        const std::map<unsigned int, math::AngDeg>& getDesiredJoints() const
            {
                return mDesiredJoints;
            }
        
    protected:
        /** 
         * protected construction for derived class
         * 
         * @param t duration
         * @param primary who create me
         */
        LowerLimbsMotion(float t, Task* primary);

        /** 
         * calculate the desired joints from given matrix of body and foots
         * 
         */
        bool calJoints(std::map<unsigned int, math::AngDeg>& angles) const;

        /** 
         * modify the move foot to avoid collide the support foot
         * 
         * @param moveFoot the matrix of moving foot
         * @param supFoot the matrix of support foot
         */
        void avoidFootCollide(math::TransMatrixf& moveFoot,
                              const math::TransMatrixf& supFoot);

    protected:
        
        /// the desired rotation and position of body, left foot and right foot
        math::TransMatrixf mDesiredBody;
        math::TransMatrixf mDesiredFootL;
        math::TransMatrixf mDesiredFootR;
        std::map<unsigned int, math::AngDeg> mDesiredJoints;
        
        // this class handls logging
        DECLARE_STATIC_GRAPHIC_LOGGER;

    };
    
} //namespace task

#endif // TASK_LOWER_LIMBS_MOTION_H
