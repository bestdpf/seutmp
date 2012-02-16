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
    using namespace std;

    SwingFoot::SwingFoot(bool isLeft,
                         const Vector2f& p,
                         float footHeight,
                         AngDeg ang,
                         AngDeg rotateFoot,
                         float bodyHeight,
                         float duration,
                         Task* primary)
        :LowerLimbsMotion(duration, primary),
         mIsLeft(isLeft)
    {
       if ( mIsLeft ){
            // left foot
            mDesiredFootL.rotationZ(ang);
            mDesiredFootL.p() = Vector3f(p.x(),p.y(),footHeight);
            // right foot
            mDesiredFootR.identity();
        }
        else{
            // right foot
            mDesiredFootR.rotationZ(ang);
            mDesiredFootR.p() = Vector3f(p.x(),p.y(),footHeight);
            // left foot
            mDesiredFootL.identity();
        }

        mDesiredFootL.p().x() -= HUMANOID.getHalfFeetWidth();
        mDesiredFootR.p().x() += HUMANOID.getHalfFeetWidth();
        mDesiredFootL.p().z() += HUMANOID.getMinFootHeight();
        mDesiredFootR.p().z() += HUMANOID.getMinFootHeight();

        // body
        AngDeg dirR = mDesiredFootR.rotatedAngZ();
        AngDeg dirL = mDesiredFootL.rotatedAngZ();
        if ( dirR > dirL ){
            std::swap(dirR, dirL);
        }
        AngDeg dirBody = calBisectorTwoAngles(dirR,dirL);
	mDesiredBody.rotationX(-10);
        mDesiredBody.rotateLocalZ( dirBody );
        mDesiredBody.pos() = (mDesiredFootL.p()+mDesiredFootR.p())*0.5f;
        // mDesiredBody.pos().x() = 0;
        mDesiredBody.pos().z() = bodyHeight;
	//dynamic step
	//mDesiredBody.p().y()-=addStepY;

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
