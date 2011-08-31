/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "HingeJoint.h"

namespace robot {
    namespace device {
        namespace joint {

            HingeJoint::HingeJoint()
            {
                setName("HingeJoint");
                mAxis.resize(1);
            }

            HingeJoint::~HingeJoint()
            {
            }

            void HingeJoint::setAxis(int axis)
            {
                setAxis( mapAxis( axis ) );
            }

            void HingeJoint::setAxis(const Vector3f& axis)
            {
                mAxis[0].axis = axis;
            }

            void HingeJoint::forwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const
            {
                const DegreeOfFreedom& dof = mAxis[0];
                AngDeg ang = angles.find(dof.id)->second;
                // rotate angle alone the axis
                m.transfer(dof.axis,ang);
            }

            void HingeJoint::backwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const
            {
                const DegreeOfFreedom& dof = mAxis[0];
                AngDeg ang = angles.find(dof.id)->second;
                // rotate angle alone the axis
                m.transfer(dof.axis,-ang);
            }

        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */
