/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_JOINT_UNIVERSALJOINT_H_
#define _ROBOT_DEVICE_JOINT_UNIVERSALJOINT_H_

#include "Joint.h"

namespace robot {
    namespace device {
        namespace joint {
            class UniversalJoint : public Joint
            {
            public:
                UniversalJoint();
    
                virtual ~UniversalJoint();

                void setAxis1(int axis);

                void setAxis2(int axis);
                
                void setAxis1(const Vector3f& axis);

                void setAxis2(const Vector3f& axis);

                virtual void forwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const;

                virtual void backwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const;
            };

        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_JOINT_UNIVERSALJOINT_H_ */
