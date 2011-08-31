/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "core/WorldModel.h"
#include "LoadFoot.h"

namespace task{
    using namespace math;

    LoadFoot::LoadFoot(bool isLeft,
                       const Vector3f& p,
                       AngDeg ang,
                       float bodyHeight,
                       float duration,
                       Task* primary)
        :MoveFoot(isLeft, p, ang, duration, primary)
    {
        AngDeg dirR = mDesiredFootR.rotatedAngZ();
        AngDeg dirL = mDesiredFootL.rotatedAngZ();
        if ( dirR > dirL ){
            std::swap(dirR, dirL);
        }
        AngDeg dirBody = calBisectorTwoAngles(dirR,dirL);
        mDesiredBody.rotationX( -10 ); // pitch
        mDesiredBody.rotateLocalZ( dirBody );
        mDesiredBody.p() = (mDesiredFootR.p() + mDesiredFootL.p()) *0.5f;
        mDesiredBody.p().x() = 0;
        mDesiredBody.pos().z() = bodyHeight;
    }
    
} // namespace task
