/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "VisionSensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            VisionSensor::VisionSensor()
            {
                setName("VisionPerceptor");
            }

            VisionSensor::~VisionSensor()
            {
            }

            void VisionSensor::setSenseMyPos(bool set)
            {
                mSenseMyPos = set;
            }

            void VisionSensor::setSenseMyTrans(bool set)
            {
                mSenseMyTrans = set;
            }

            void VisionSensor::setStaticSenseAxis(bool set)
            {
                mStaticSenseAxis = set;
            }

            void VisionSensor::addNoise(bool set)
            {
                mAddNoise = set;
            }

            RestrictedVisionSensor::RestrictedVisionSensor()
            {
                setName("RestrictedVisionSensor");
                setViewCones(90,90);
            }

            RestrictedVisionSensor::~RestrictedVisionSensor()
            {
            }

            void RestrictedVisionSensor::setViewCones(float hAngle, float vAngle)
            {
                mMaxHorizotalAngle = hAngle;
                mMaxVerticalAngle = vAngle;
            }
            
        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */
