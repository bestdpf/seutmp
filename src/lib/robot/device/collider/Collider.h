/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_COLLIDER_COLLIDER_H_
#define _ROBOT_DEVICE_COLLIDER_COLLIDER_H_

#include "../Device.h"


namespace robot {
    namespace device {
        namespace collider {

            class Collider : public Device
            {
            public:
                Collider()
                    {
                        mLocalMat.identity();
                    }
                
                void addNotCollideWithColliderName(const std::string& name,bool isAdd)
                    {
                        if ( isAdd ){
                            mNotCollideWithSet.insert(name);
                        }
                        else{
                            mNotCollideWithSet.erase(name);
                        }
                    }
                
                
                void setLocalPosition(const Vector3f& p)
                    {
                        mLocalMat.p() = p;
                    }

                void setRotation(const Vector3f& r)
                    {
                        mLocalMat.rotationX(r.x());
                        mLocalMat.rotateLocalY(r.y());
                        mLocalMat.rotateLocalZ(r.z());
                    }

                /** 
                 * return ZERO, the size function should depend on
                 * inherited collider
                 * 
                 * @return the size of the collider 
                 */
                virtual Vector3f size() const { return Vector3f(0,0,0); }
                
            protected:
                set<string> mNotCollideWithSet;
                
                TransMatrixf mLocalMat;
            };
    
        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_COLLIDER_COLLIDER_H_ */
