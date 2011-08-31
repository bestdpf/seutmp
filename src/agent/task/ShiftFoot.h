/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _TASK_SHIFTFOOT_H_
#define _TASK_SHIFTFOOT_H_

#define ENABLE_TASK_SHIFT_FOOT_LOG

#include "MoveFoot.h"
#ifdef ENABLE_TASK_SHIFT_FOOT_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{

class ShiftFoot : public MoveFoot
{
public:
    ShiftFoot(bool isLeft,
              float bodyHeight,
              float duration,
              Task* primary);

    virtual void updateSubTaskList();
    
private:
    bool mIsStarted;
    float mBodyHeight;
    
    // this class handls logging
    DECLARE_STATIC_GRAPHIC_LOGGER;
};

} // namespace task
    
#endif /* _TASK_SHIFTFOOT_H_ */
