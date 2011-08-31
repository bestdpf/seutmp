/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_COLLIDER_SPHERECOLLIDER_H_
#define _ROBOT_DEVICE_COLLIDER_SPHERECOLLIDER_H_

#include "Collider.h"

namespace robot {
    namespace device {
        namespace collider {

            class SphereCollider : public Collider
            {
            public:

                SphereCollider();

                void setRadius(float r);

                virtual Vector3f size() const;

            private:
                float mRadius;
            };
            

        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */

#endif /* _ROBOT_DEVICE_COLLIDER_SPHERECOLLIDER_H_ */
