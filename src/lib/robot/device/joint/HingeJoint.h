/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_JOINT_HINGEJOINT_H_
#define _ROBOT_DEVICE_JOINT_HINGEJOINT_H_

#include "Joint.h"

namespace robot {
    namespace device {
        namespace joint {

            class HingeJoint : public Joint
            {
            public:
                HingeJoint();
        
                virtual ~HingeJoint();

                void setAxis(int axis);

                void setAxis(const Vector3f& axis);

                virtual void forwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const;

                virtual void backwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const;
                
            };
    
        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_JOINT_HINGEJOINT_H_ */
