/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "FixedJoint.h"

namespace robot {
    namespace device {
        namespace joint {

            FixedJoint::FixedJoint()
            {
                setName("FixedJoint");
            }

            FixedJoint::~FixedJoint()
            {
            }

            void FixedJoint::setFixed()
            {
            }
            
        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */
