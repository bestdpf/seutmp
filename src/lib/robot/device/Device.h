/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_DEVICE_H_
#define _ROBOT_DEVICE_DEVICE_H_

#include "math/Math.hpp"
#include "Node.hpp"
#include "Template.hpp"

namespace robot {
    namespace device {
        using namespace std;
        using namespace boost;
        using namespace math;
        using namespace tree;
        
        class Device
        {
        public:
            
            Device():mName("?Device"){};

            virtual ~Device() 
            {
            }
            
            const string& getName() const;

            void setName(const string& name);

        private:
            string mName;
        };
        
    } /* namespace device */
} /* namespace robot */
#endif /* _ROBOT_DEVICE_DEVICE_H_ */
