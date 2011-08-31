/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_RIGIDBODY_H_
#define _ROBOT_DEVICE_RIGIDBODY_H_

#include "Device.h"

namespace robot {
    namespace device {
        class RigidBody : public Device
        {
        public:
            RigidBody():mMass(0),mSize(0,0,0),mMassTrans(0,0,0)
                {
                }
            
            /** 
             * the reference to mass of body 
             * use this to set the mass
             * @return the mass' reference
             */
            float& mass();

            /** 
             * get the mass of body
             * @return mass 
             */
            float mass() const;

            /** 
             * this body is set as a box
             * 
             * @param mass the mass of the box
             * @param size the size of the box
             */
            void setBox(float mass, const Vector3f& size);

            void setBoxTotal(float mass, const Vector3f& size);

            /** 
             * this body is set as a sphere
             * 
             * @param mass the mass of the sphere
             * @param radius the radius of the sphere
             */
            void setSphere(float mass, float radius);

            void setSphereTotal(float mass, float radius);
            
            void setCapsuleTotal(float mass, float radius, float length);

            /** 
             * add a new box to construct a composite body
             * 
             * @param mass the added box's mass
             * @param size the size of added box
             * @param p the relative position of the added box
             * @param rot the rotation angles of the added box
             */
            void addBox(float mass, const Vector3f& size, const Vector3f& p, const Vector3f& rot);

            const Vector3f& size() const
                {
                    return mSize;
                }
            
        protected:
            
            /** mass of this body */
            float mMass;

            /// the size of this body
            Vector3f mSize;

            /// the total mass translation
            Vector3f mMassTrans;
        };
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_RIGIDBODY_H_ */
