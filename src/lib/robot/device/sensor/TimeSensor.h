/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_SENSOR_TIMESENSOR_H_
#define _ROBOT_DEVICE_SENSOR_TIMESENSOR_H_

#include "Sensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            class TimeSensor : public Sensor
            {
            public:
                TimeSensor();
        
                virtual ~TimeSensor();
            };
    
    
        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_SENSOR_TIMESENSOR_H_ */
