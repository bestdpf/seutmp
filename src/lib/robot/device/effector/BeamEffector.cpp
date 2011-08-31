/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "BeamEffector.h"
#include <boost/lexical_cast.hpp>

namespace robot {
    namespace device {
        namespace effector {
            using namespace std;
            using namespace boost;

            BeamEffector::BeamEffector()
            {
                setName("BeamEffector");
            }

            BeamEffector::~BeamEffector()
            {
            }
            
        } /* namespace effector */
    } /* namespace device */
} /* namespace robot */
