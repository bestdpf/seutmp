/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_COLLIDER_CAPSULECOLLIDER_H_
#define _ROBOT_DEVICE_COLLIDER_CAPSULECOLLIDER_H_

#include "Collider.h"
namespace robot {
    namespace device {
        namespace collider {

            class CapsuleCollider : public Collider
            {
            public:

                CapsuleCollider();

                void setParams(float radius, float length);

            private:
                float mRadius;
                float mLength;
                
            };
    
        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_COLLIDER_CAPSULECOLLIDER_H_ */
