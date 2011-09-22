/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "MoveFoot.h"
#include "core/WorldModel.h"
#include "robot/humanoid/Humanoid.h"

namespace task{

    using namespace math;
    using namespace serversetting;

    Vector2f MoveFoot::mPitchRatio( 0, 1);
    
    MoveFoot::MoveFoot(bool isLeft,
                       const Vector3f& p,
                       AngDeg ang,
                       float duration,
                       Task* primary)
        :LowerLimbsMotion(duration, primary),
         mIsLeft(isLeft)
    {
        if ( mIsLeft ){
            // left foot
            mDesiredFootL.rotationZ(ang);
            mDesiredFootL.p() = p;
            // right foot
            mDesiredFootR.identity();
        }
        else{
            // right foot
            mDesiredFootR.rotationZ(ang);
            mDesiredFootR.p() = p;
            // left foot
            mDesiredFootL.identity();
        }

        mDesiredFootL.p().x() -= HUMANOID.getHalfFeetWidth();
        mDesiredFootR.p().x() += HUMANOID.getHalfFeetWidth();
        mDesiredFootL.p().z() += HUMANOID.getMinFootHeight();
        mDesiredFootR.p().z() += HUMANOID.getMinFootHeight();

   }
   
} // namespace task
