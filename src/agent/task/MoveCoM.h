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
 * @file   MoveCoM.h
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Sun Oct  7 02:49:24 2007
 * 
 * @brief  It contains squat and move COM to desired position while two
 * foot supporting. It is can be used in many suitation,
 * such as perform preparation for walking
 *
 * @note   currently assume the CoM is fixed with the body of the robot
 * but it is NOT true.
 */


#ifndef TASK_MOVE_COM_H
#define TASK_MOVE_COM_H

#include "LowerLimbsMotion.h"

namespace task{

    class MoveCoM: public LowerLimbsMotion
    {
    public:
        /** 
         * create the move CoM task
         * 
         * @param target the desired position of CoM ( center of mass ) in local
         * @param duration allowed the duration
         * @param primary who creates me
         * 
         */
        MoveCoM(const math::Vector3f& target, float duration, Task* primary);

        virtual void updateSubTaskList();
        
    private:
        /// the target local position of CoM
        math::Vector3f mTarget;

        bool mIsStarted;
    };
    
} //namespace task

#endif // TASK_WALK_START_H
