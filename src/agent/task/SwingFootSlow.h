/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _TASK_SWINGFOOT_H_
#define _TASK_SWINGFOOT_H_

#include "MoveFoot.h"

namespace task{

    class SwingFootSlow : public MoveFoot
    {
    public:
        SwingFootSlow(bool isLeft,
                  const math::Vector2f& p,
                  float footHeight,
                  math::AngDeg ang,
                  math::AngDeg rotateFoot,
                  float bodyHeight,
                  float duration,
                  Task* primary);
    };

} // namespace task

#endif /* _TASK_SWINGFOOT_H_ */
