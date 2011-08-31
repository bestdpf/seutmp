/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "RigidBody.h"

namespace robot {
    namespace device {
        float& RigidBody::mass()
        {
            return mMass;
        }

        float RigidBody::mass() const
        {
            return mMass;
        }

        void RigidBody::setBox(float mass, const Vector3f& size)
        {
            mMass = mass;
            mSize = size;
        }

        void RigidBody::setBoxTotal(float mass, const Vector3f& size)
        {
            mMass = mass;
            mSize = size;
        }

        void RigidBody::setSphere(float mass, float radius)
        {
            mMass = mass;
            mSize = Vector3f(radius,radius,radius);
        }

        void RigidBody::setSphereTotal(float mass, float radius)
        {
            mMass = mass;
            mSize = Vector3f(radius,radius,radius);
        }

        void RigidBody::setCapsuleTotal(float mass, float radius, float length)
        {
			mMass = mass;
			mSize = Vector3f(radius, radius, length);
        }

        void RigidBody::addBox(float mass, const Vector3f& /*size*/, const Vector3f& p, const Vector3f& /*rot*/)
        {
            // @NOTE@ just skip the rotation here!!!
            float massTotal = mass + mMass;
            mMassTrans = p*mass + mMassTrans*mMass;
            mMass = massTotal;
        }
        
    } /* namespace device */
} /* namespace robot */
