/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_JOINT_FIXEDJOINT_H_
#define _ROBOT_DEVICE_JOINT_FIXEDJOINT_H_

#include "Joint.h"

namespace robot {
    namespace device {
        namespace joint {

            class FixedJoint : public Joint
            {
            public:
                FixedJoint();
        
                virtual ~FixedJoint();

                void setFixed();

                virtual void forwardKinematics(TransMatrixf& /*m*/, const map<unsigned int, AngDeg>& /*angles*/) const
                    { /* do nothing */ }
                
                virtual void backwardKinematics(TransMatrixf& /*m*/, const map<unsigned int, AngDeg>& /*angles*/) const
                    { /* do nothing */ }
            };    
    
        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_JOINT_FIXEDJOINT_H_ */
