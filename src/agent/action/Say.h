/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef _ACTION_SAY_H_
#define _ACTION_SAY_H_

#include "Action.h"

namespace action{

    class Say : public Action
    {
    public:
        Say(const std::string& msg)
            :mMsg(msg)
            {}

        virtual ~Say(){}

        virtual std::string command() const;
    private:
        std::string mMsg;
    };
    
} // namespace action

#endif /* _ACTION_SAY_H_ */
