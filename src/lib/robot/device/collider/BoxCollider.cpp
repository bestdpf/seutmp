/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "BoxCollider.h"

namespace robot {
    namespace device {
        namespace collider {

            BoxCollider::BoxCollider()
            {
                setName("BoxCollider");
            }
            
            void BoxCollider::setBoxLengths(const Vector3f& len)
            {
                mLengths = len;
            }

            Vector3f BoxCollider::size() const
            {
                return mLengths;
            }
        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */
