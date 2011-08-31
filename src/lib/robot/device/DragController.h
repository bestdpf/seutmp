/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_DRAGCONTROLLER_H_
#define _ROBOT_DEVICE_DRAGCONTROLLER_H_

#include "Device.h"

namespace robot {
    namespace device {

        class DragController : public Device
        {
        public:

            DragController();
            
            void setAngularDrag(float d);

            void setLinearDrag(float d);

        private:
            float 	mLinearDrag;
            float 	mAngularDrag;
        };
     
    } /* namespace device */   
} /* namespace robot */


#endif /* _ROBOT_DEVICE_DRAGCONTROLLER_H_ */
