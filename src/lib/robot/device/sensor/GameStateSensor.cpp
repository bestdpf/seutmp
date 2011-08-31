/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "GameStateSensor.h"

namespace robot {
    namespace device {
        namespace sensor {

            GameStateSensor::GameStateSensor()
            {
                setName("GameStatePerceptor");
            }

            GameStateSensor::~GameStateSensor()
            {
            }
            
        } /* namespace sensor */
    } /* namespace device */
} /* namespace robot */
