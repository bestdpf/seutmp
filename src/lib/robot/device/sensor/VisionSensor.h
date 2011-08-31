/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_SENSOR_VISIONSENSOR_H_
#define _ROBOT_DEVICE_SENSOR_VISIONSENSOR_H_

#include "Sensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            class VisionSensor : public Sensor
            {
            public:
                VisionSensor();
        
                virtual ~VisionSensor();

                void setSenseMyPos(bool set);

                void setSenseMyTrans(bool set);

                void setStaticSenseAxis(bool set);

                void addNoise(bool set);

            private:
                bool mSenseMyPos;
                bool mSenseMyTrans;
                bool mStaticSenseAxis;
                bool mAddNoise;
            };

            class RestrictedVisionSensor : public VisionSensor
            {
            public:
                RestrictedVisionSensor();
                virtual ~RestrictedVisionSensor();

                /** 
                 * set the range of vision sensor
                 * 
                 * @param hAngle the max horizontal angle
                 * @param vAngle the max vertical angle
                 */
                void setViewCones(float hAngle, float vAngle);
            private:
                float mMaxHorizotalAngle;
                float mMaxVerticalAngle;
            };
    
        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */



#endif /* _ROBOT_DEVICE_SENSOR_VISIONSENSOR_H_ */
