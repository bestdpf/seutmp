/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_EFFECTOR_HINGEJOINTEFFECTOR_H_
#define _ROBOT_DEVICE_EFFECTOR_HINGEJOINTEFFECTOR_H_

#include "Effector.h"

namespace robot {
    namespace device {
        namespace effector {
            using namespace std;

            /**
             * the effector for hinge joint
             * 
             */
            class HingeJointEffector : public Effector
            {
            public:
                HingeJointEffector();
                virtual ~HingeJointEffector();
            };
        } /* namespace effector */
    } /* namespace  device */
} /* namespace  robot */



#endif /* _ROBOT_DEVICE_EFFECTOR_HINGEJOINTEFFECTOR_H_ */
