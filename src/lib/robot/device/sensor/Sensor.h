/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_SENSOR_SENSOR_H_
#define _ROBOT_DEVICE_SENSOR_SENSOR_H_

#include "../Device.h"

namespace robot {
    namespace device {
        namespace sensor {

            class Sensor : public Device
            {
            public:
                Sensor():mInterval(1){};

                /** 
                 * set the interval of sense
                 * 
                 * @param i the interval number
                 */
                void setInterval(unsigned int i){mInterval=i;}

            private:
                unsigned int mInterval;
            };
            
        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */

#endif /* _ROBOT_DEVICE_SENSOR_SENSOR_H_ */
