/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "CapsuleCollider.h"

namespace robot {
    namespace device {
        namespace collider {

            CapsuleCollider::CapsuleCollider()
            {
                setName("CapsuleCollider");
            }

            void CapsuleCollider::setParams(float radius, float length)
            {
                mRadius = radius;
                mLength = length;
            }
    
        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */
