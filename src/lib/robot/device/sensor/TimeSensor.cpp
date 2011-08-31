/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "TimeSensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            TimeSensor::TimeSensor()
            {
                setName("TimePerceptor");
            }

            TimeSensor::~TimeSensor()
            {
            }
    
        } /* namespace sensor */

    } /* namespace device */

} /* namespace robot */
