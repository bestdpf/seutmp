/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "MoveCoM.h"
#include "core/WorldModel.h"

namespace task{

    using namespace math;
    using namespace serversetting;
	using namespace perception;
    
    MoveCoM::MoveCoM(const Vector3f& target, float duration, Task* primary)
        :LowerLimbsMotion(duration, primary),
         mTarget(target),
         mIsStarted(false)
    {
        mDesiredFootR.identity();
        mDesiredFootR.p().x() = HUMANOID.getHalfFeetWidth();
        mDesiredFootR.p().z() = HUMANOID.getMinFootHeight();
        mDesiredFootL.identity();
        mDesiredFootL.p().x() = -HUMANOID.getHalfFeetWidth();
        mDesiredFootL.p().z() = HUMANOID.getMinFootHeight();
        mDesiredBody.rotationX(-10);
        mDesiredBody.pos() = mTarget;
    }

    void MoveCoM::updateSubTaskList()
    {
        if ( !mIsStarted ) {
            TransMatrixf tempMat;
            const TransMatrixf& myOigMat = WM.getMyOriginTrans();
            // body
            mDesiredBody = myOigMat;
            // AngDeg dirBody = WM.getVisionTrans().rotatedAngZ();
            // tempMat.rotationZ( dirBody );
            tempMat = WM.getBoneTrans(robot::humanoid::Humanoid::TORSO);
            mDesiredBody.inverseTransfer( tempMat );
            mDesiredBody.pos() = mTarget;
        
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
            mIsStarted = true;
        }

        LowerLimbsMotion::updateSubTaskList();
    }
    
} // namespace task
