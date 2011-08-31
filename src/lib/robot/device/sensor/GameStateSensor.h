/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_SENSOR_GAMESTATESENSOR_H_
#define _ROBOT_DEVICE_SENSOR_GAMESTATESENSOR_H_

#include "Sensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            class GameStateSensor : public Sensor
            {
            public:
                GameStateSensor();
        
                virtual ~GameStateSensor();
            };
    
    
        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_SENSOR_GAMESTATESENSOR_H_ */
