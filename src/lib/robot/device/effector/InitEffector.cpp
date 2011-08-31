/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "InitEffector.h"
#include <boost/lexical_cast.hpp>

namespace robot {
    namespace device {
        namespace effector {
            using namespace std;
            using namespace boost;

            InitEffector::InitEffector()
            {
                setName("InitEffector");
            }

            InitEffector::~InitEffector()
            {
            }
            
        } /* namespace effector */
    } /* namespace device */
} /* namespace robot */
