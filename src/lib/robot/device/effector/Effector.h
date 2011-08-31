/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_EFFECTOR_EFFECTOR_H_
#define _ROBOT_DEVICE_EFFECTOR_EFFECTOR_H_

#include "../Device.h"

namespace robot {
    namespace device {
        namespace effector {
            using namespace std;
            
            /**
             * abstract effector class of robot
             * 
             */
            class Effector : public Device
            {
            public:
            };
            
        } /* namespace effector */
    } /* namespace  device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_EFFECTOR_EFFECTOR_H_ */
