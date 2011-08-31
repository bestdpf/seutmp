/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_SPACE_H_
#define _ROBOT_DEVICE_SPACE_H_

#include "Device.h"

namespace robot {
    namespace device {

        class Space : public Device
        {
        public:
            Space();
            
            virtual ~Space();

            void disableInnerCollision(bool set);

        private:
            bool mDisableInnerCollision;
        };
    
    } /* namespace device */
} /* namespace robot */

#endif /* _ROBOT_DEVICE_SPACE_H_ */
