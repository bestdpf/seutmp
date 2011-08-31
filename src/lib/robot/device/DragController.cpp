/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "DragController.h"

namespace robot {
    namespace device {

        DragController::DragController()
        {
            setName("DragController");
        }

        void DragController::setAngularDrag(float d)
        {
            mAngularDrag = d;
        }

        void DragController::setLinearDrag(float d)
        {
            mLinearDrag = d;
        }
    
    } /* namespace device */
} /* namespace robot */
