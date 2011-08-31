/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_EFFECTOR_INITEFFECTOR_H_
#define _ROBOT_DEVICE_EFFECTOR_INITEFFECTOR_H_

#include "Effector.h"

namespace robot {
    namespace device {
        namespace effector {
            using namespace std;
            
            class InitEffector : public Effector
            {
            public:
                InitEffector();
    
                virtual ~InitEffector();
            };

            class SingleMatInitEffector : public InitEffector
            {
            public:
                SingleMatInitEffector(){};
                virtual ~SingleMatInitEffector(){};
            };

            class StaticMeshInitEffector : public InitEffector
            {
            public:
                StaticMeshInitEffector(){};
                virtual ~StaticMeshInitEffector(){};
            };

        } /* namespace effector */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_EFFECTOR_INITEFFECTOR_H_ */ 
