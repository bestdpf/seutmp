/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Say.h"
#include "core/WorldModel.h"

namespace action{

    std::string Say::command() const
    {
        std::stringstream ss;
        ss<<"(say "
          <<((WM.getOurTeamIndex()==serversetting::TI_LEFT)?'L':'R')
        //  << int(WM.getGameTime()*100) %100 <<","
           //     << 0 << ","
                <<'S'
          << mMsg <<")";
        return ss.str();
    }
    
} // namespace action

