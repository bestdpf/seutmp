/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_COLLIDER_BOXCOLLIDER_H_
#define _ROBOT_DEVICE_COLLIDER_BOXCOLLIDER_H_

#include "Collider.h"

namespace robot {
    namespace device {
        namespace collider {

            class BoxCollider : public Collider
            {
            public:

                BoxCollider();
                
                void setBoxLengths(const Vector3f& len);

                virtual Vector3f size() const;

            private:
                Vector3f mLengths;
            };
    
        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_COLLIDER_BOXCOLLIDER_H_ */
