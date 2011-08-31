/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_EFFECTOR_UNIVERSALJOINTEFFECTOR_H_
#define _ROBOT_DEVICE_EFFECTOR_UNIVERSALJOINTEFFECTOR_H_

#include "Effector.h"
namespace robot {
    namespace device {
        namespace effector {
            class UniversalJointEffector : public Effector
            {
            public:
                UniversalJointEffector();
                
                virtual ~UniversalJointEffector();
            };
            
        } /* namespace effector */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_EFFECTOR_UNIVERSALJOINTEFFECTOR_H_ */
