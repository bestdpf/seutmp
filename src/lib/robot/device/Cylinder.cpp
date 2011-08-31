/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Cylinder.h"

namespace robot {
    namespace device {

        Cylinder::Cylinder() {
            setName("Cylinder");
        }

        void Cylinder::setParams(float radius, float length) {
            mRadius = radius;
            mLength = length;
        }

        void Cylinder::setScale(float x,float y, float z ){
            mX=x;
            mY=y;
            mZ=z;
        }

            void Cylinder::setMaterial(const string& material){
                mMaterial=material;
            }
            void Cylinder::setTransparent()
            {

            }


    } /* namespace device */
} /* namespace robot */
