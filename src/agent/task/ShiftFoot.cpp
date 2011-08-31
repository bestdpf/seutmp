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
#include "configuration/Configuration.h"
#include "ShiftFoot.h"
#include "robot/humanoid/Humanoid.h"

namespace task{
    using namespace math;
    DEFINE_STATIC_GRAPHIC_LOGGER(ShiftFoot)
    
    ShiftFoot::ShiftFoot(bool isLeft,
                         float bodyHeight,
                         float duration,
                         Task* primary)
    :MoveFoot(isLeft, duration, primary),
        mIsStarted(false),
        mBodyHeight(bodyHeight)
    {
        BEGIN_ADD_STATIC_LOG_LAYER(ShiftFoot)
        ADD_LOG_LAYER("new");
        END_ADD_STATIC_LOG_LAYER(ShiftFoot)

            LOG_PRINTF("new","isLeft=%d, bodyHeight=%.3f, duration=%.3f",
                       isLeft, bodyHeight, duration);
        LOG_FLUSH;
    }

    void ShiftFoot::updateSubTaskList()
    {
        if ( !mIsStarted) {
            TransMatrixf tempMat;
            const TransMatrixf& myOigMat = WM.getMyOriginTrans();
        
            // left foot
            mDesiredFootL = myOigMat;
            const TransMatrixf& matL = WM.getBoneTrans(robot::humanoid::Humanoid::L_FOOT);
            AngDeg dirFootL = matL.rotatedAngZ();
            tempMat.rotationZ( dirFootL );
            tempMat.p() = matL.p();
            mDesiredFootL.inverseTransfer( tempMat );
            mDesiredFootL.p().z() = HUMANOID.getMinFootHeight();
        
            // right foot
            mDesiredFootR = myOigMat;
            const TransMatrixf& matR = WM.getBoneTrans(robot::humanoid::Humanoid::R_FOOT);
            AngDeg dirFootR = matR.rotatedAngZ();
            tempMat.rotationZ( dirFootR );
            tempMat.p() = matR.p();
            mDesiredFootR.inverseTransfer( tempMat );
            mDesiredFootR.p().z() = HUMANOID.getMinFootHeight();

            // body
            AngDeg dirR = mDesiredFootR.rotatedAngZ();
            AngDeg dirL = mDesiredFootL.rotatedAngZ();
            AngDeg dirBody = mIsLeft?dirR:dirL;
            mDesiredBody.rotationX( -10 ); // pitch
            mDesiredBody.rotateLocalZ( dirBody );
            mDesiredBody.pos().x() = mIsLeft?mDesiredFootR.pos().x():mDesiredFootL.pos().x();
            mDesiredBody.pos().x() *= 0.65f;
            mDesiredBody.pos().y() = ( mDesiredFootL.pos().y() + mDesiredFootR.pos().y() ) * 0.5f;
            mDesiredBody.pos().z() = mBodyHeight;

            if (mIsLeft){
                mDesiredFootL.p().z() += HUMANOID.getMinFootHeight();
            }
            else{
                mDesiredFootR.p().z() += HUMANOID.getMinFootHeight();
            }

            mIsStarted = true;
        }

        LowerLimbsMotion::updateSubTaskList();
    }

} // name space task
