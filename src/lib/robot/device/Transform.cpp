/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "Transform.h"

namespace robot {
    namespace device {

        Transform::Transform()
        {
            setName("Transform");
            mLocalMat.identity();
        }

        Transform::~Transform()
        {
        }

        void Transform::setLocalPos(const Vector3f& pos)
        {
            mLocalMat.pos() = pos;
        }

        void Transform::setLocalRotation(const Vector3f& rot)
        {
            mLocalMat.rotationX(rot[0]);
            mLocalMat.rotateLocalY(rot[1]);
            mLocalMat.rotateLocalZ(rot[2]);
        }
        
    } /* namespace device */
} /* namespace robot */
