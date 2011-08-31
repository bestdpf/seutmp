/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef _ROBOT_DEVICE_OBJECTSTATE_H_
#define _ROBOT_DEVICE_OBJECTSTATE_H_

#include "Device.h"

namespace robot{
    namespace device{

        class ObjectState : public Device
        {
        public:
            void setID(const std::string& id){ mId = id; }
            
        private:
            std::string mId;
        };
        
    }} /* namespace robot::device */


#endif /* _ROBOT_DEVICE_OBJECTSTATE_H_ */
