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

#include "LowerLimbsMotion.h"

namespace task{

    class SwingFoot : public LowerLimbsMotion
    {
    public:
        SwingFoot(bool isLeft,
                  const math::Vector2f& p,
                  float footHeight,
                  math::AngDeg ang,
                  math::AngDeg rotateFoot,
                  float bodyHeight,
                  float duration,
                  Task* primary);
    private:
      bool mIsLeft;
    };

} // namespace task

#endif /* _TASK_SWINGFOOT_H_ */
