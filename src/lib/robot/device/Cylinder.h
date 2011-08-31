/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_CYLINDER_H_
#define _ROBOT_DEVICE_CYLINDER_H_

#include "Device.h"

namespace robot {
    namespace device {

        class Cylinder : public Device {
        public:
            Cylinder();
            void setParams(float radius, float length);//the name is not the mean of the params
            void setScale(float x,float y, float z );
            void setMaterial(const string& material);
            void setTransparent();

        private:
            float mRadius;
            float mLength;
            float mX;
            float mY;
            float mZ;
            string mMaterial;
        };


    } /* namespace device */

} /* namespace robot */


#endif /* _ROBOT_DEVICE_CYLINDER_H_ */
