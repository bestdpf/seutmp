/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: AgentSyncEffector.h,v 1.0 2010/03/15  Allen Exp $
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_EFFECTOR_AGENTSYNCEFFECTOR_H_
#define _ROBOT_DEVICE_EFFECTOR_AGENTSYNCEFFECTOR_H_

#include "Effector.h"

namespace robot {
    namespace device {
        namespace effector {

            class AgentSyncEffector : public Effector
            {
            public:
                AgentSyncEffector();

                virtual ~AgentSyncEffector();
            };

        } /* namespace effector */
    } /* namespace device */
} /* namespace robot */



#endif /* _ROBOT_DEVICE_EFFECTOR_AGENTSYNCEFFECTOR_H_ */
