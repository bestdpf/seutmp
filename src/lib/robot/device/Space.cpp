/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "Space.h"

namespace robot {
    namespace device {

        Space::Space()
        {
        }

        Space::~Space()
        {
        }

        void Space::disableInnerCollision(bool set)
        {
            mDisableInnerCollision = set;
        }
        
    } /* namespace device */
} /* namespace robot */

