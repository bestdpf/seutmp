
/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id:AccelerometerSensor.h ,v 1.0 2010/03/15  Allen Exp $
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_SENSOR_ACCELEROMETERSENSOR_H_
#define _ROBOT_DEVICE_SENSOR_ACCELEROMETERSENSOR_H_

#include "Sensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            class AccelerometerSensor : public Sensor
            {
            public:
                AccelerometerSensor();
            };


        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_SENSOR_ACCELEROMETERSENSOR_H_ */
