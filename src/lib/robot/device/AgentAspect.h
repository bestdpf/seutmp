/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_AGENTASPECT_H_
#define _ROBOT_DEVICE_AGENTASPECT_H_

#include "Transform.h"


namespace robot {
    namespace device {

        class AgentAspect : public Transform
        {
        public:
            AgentAspect();
        
            virtual ~AgentAspect();
        };    
    
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_AGENTASPECT_H_ */
