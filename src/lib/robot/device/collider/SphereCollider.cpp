/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "SphereCollider.h"

namespace robot {
namespace device {
namespace collider {

    SphereCollider::SphereCollider()
    {
        setName("SphereCollider");
    }

    void SphereCollider::setRadius(float r)
    {
        mRadius = r;
    }

    Vector3f SphereCollider::size() const
    {
        return Vector3f(mRadius,mRadius,mRadius);
    }
    
} /* namespace collider */
} /* namespace device */
} /* namespace robot */
