/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "SwingFoot.h"
#include "Template.hpp"
#include "robot/humanoid/Humanoid.h"
#include "controller/ArmMotion.h"
#include "core/WorldModel.h"
#include "action/Actions.h"

namespace task{
    using namespace math;

    SwingFoot::SwingFoot(bool isLeft,
                         const Vector2f& p,
                         float footHeight,
                         AngDeg ang,
                         AngDeg rotateFoot,
                         float bodyHeight,
                         float duration,
                         Task* primary)
        :MoveFoot(isLeft, Vector3f(p.x(),p.y(),footHeight), ang, duration, primary)
    {
        // body
        AngDeg dirR = mDesiredFootR.rotatedAngZ();
        AngDeg dirL = mDesiredFootL.rotatedAngZ();
        if ( dirR > dirL ){
            std::swap(dirR, dirL);
        }
        AngDeg dirBody = calBisectorTwoAngles(dirR,dirL);
        mDesiredBody.rotationX( -10 ); // pitch
        mDesiredBody.rotateLocalZ( dirBody );
        mDesiredBody.pos() = (mDesiredFootL.p()+mDesiredFootR.p())*0.5f;
        // mDesiredBody.pos().x() = 0;
        mDesiredBody.pos().z() = bodyHeight;

        if ( p.y()>0 ){
            if(mIsLeft){
                mDesiredFootL.rotateLocalX(rotateFoot);
            }
            else{
                mDesiredFootR.rotateLocalX(rotateFoot);
            }
        }
    }

} // namespace task
