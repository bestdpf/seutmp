/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_EFFECTOR_SAYEFFECTOR_H_
#define _ROBOT_DEVICE_EFFECTOR_SAYEFFECTOR_H_

#include "Effector.h"

namespace robot {
    namespace device {
        namespace effector {

            class SayEffector : public Effector
            {
            public:
                SayEffector();
        
                virtual ~SayEffector();
            };
            
        } /* namespace effector */
    } /* namespace device */
} /* namespace robot */



#endif /* _ROBOT_DEVICE_EFFECTOR_SAYEFFECTOR_H_ */
