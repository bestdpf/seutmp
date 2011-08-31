/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/


#include "Device.h"

namespace robot {
    namespace device {
        
        const string& Device::getName() const
        {
            return mName;
        }

        void Device::setName(const string& name)
        {
            mName = name;
        }
        
    } /* namespace device */
} /* namespace robot */

