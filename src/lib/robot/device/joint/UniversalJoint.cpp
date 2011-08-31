/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "UniversalJoint.h"

namespace robot {
    namespace device {
        namespace joint {

            UniversalJoint::UniversalJoint()
            {
                setName("UniversalJoint");
                mAxis.resize(2);
            }

            UniversalJoint::~UniversalJoint()
            {
            }

            void UniversalJoint::setAxis1(const Vector3f& axis)
            {
                mAxis[0].axis = axis;
            }

            void UniversalJoint::setAxis2(const Vector3f& axis)
            {
                mAxis[1].axis = axis;
            }

            void UniversalJoint::setAxis1( int axis )
            {
                setAxis1( mapAxis( axis ) );
            }

            void UniversalJoint::setAxis2( int axis )
            {
                setAxis2( mapAxis( axis ) );
            }

            void UniversalJoint::forwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const
            {
                const DegreeOfFreedom& dof0 = mAxis[0];
                const DegreeOfFreedom& dof1 = mAxis[1];
                AngDeg ang0 = angles.find(dof0.id)->second;
                AngDeg ang1 = angles.find(dof1.id)->second;
                // rotate angle alone the axis
                m.transfer(dof0.axis,ang0);
                m.transfer(dof1.axis,ang1);
            }

            void UniversalJoint::backwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const
            {
                const DegreeOfFreedom& dof0 = mAxis[0];
                const DegreeOfFreedom& dof1 = mAxis[1];
                AngDeg ang0 = angles.find(dof0.id)->second;
                AngDeg ang1 = angles.find(dof1.id)->second;
                // rotate angle alone the axis
                m.transfer(dof1.axis,-ang1);
                m.transfer(dof0.axis,-ang0);
            }

        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */
