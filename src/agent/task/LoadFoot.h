/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _TASK_LOADFOOT_H_
#define _TASK_LOADFOOT_H_

#include "MoveFoot.h"

namespace task{

    class LoadFoot : public MoveFoot
    {
    public:
        LoadFoot(bool isLeft,
                 const math::Vector3f& p,
                 math::AngDeg ang,
                 float bodyHeight,
                 float duration,
                 Task* primary);
    private:
    };

    
} // namespace task


#endif /* _TASK_LOADFOOT_H_ */
