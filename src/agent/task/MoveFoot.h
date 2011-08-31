/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef TASK_MOVE_FOOT_H
#define TASK_MOVE_FOOT_H

#include "LowerLimbsMotion.h"

namespace task{

    class MoveFoot: public LowerLimbsMotion
    {
    public:
        MoveFoot(bool isLeft,
                 const math::Vector3f& p,
                 math::AngDeg ang,
                 float duration,
                 Task* primary);

        MoveFoot(bool isLeft, float duration, Task* primary);

        static void setPitchRatio( const math::Vector2f& v )
            {
                mPitchRatio = v;
            }

        static const math::Vector2f& getPitchRatio()
            {
                return mPitchRatio;
            }
        
    protected:
        /// which foot moving, the other is support foot
        bool mIsLeft;
        
        /// the target local transform matrix of the foot
        math::TransMatrixf mTarget;

        static math::Vector2f mPitchRatio;
    };
    
} //namespace task

#endif // TASK_MOVE_FOOT_H
