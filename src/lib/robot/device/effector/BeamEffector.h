/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_EFFECTOR_BEAMEFFECTOR_H_
#define _ROBOT_DEVICE_EFFECTOR_BEAMEFFECTOR_H_

#include "Effector.h"
#include "math/Math.hpp"

namespace robot {
    namespace device {
        namespace effector {
            using namespace std;
            using namespace math;

            class BeamEffector : public Effector
            {
            public:
                BeamEffector();
                
                virtual ~BeamEffector();
            };
            
        } /* namespace effector */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_EFFECTOR_BEAMEFFECTOR_H_ */
