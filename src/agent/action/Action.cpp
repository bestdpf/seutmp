/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Action.cpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#include "Action.h"
#include <ostream>

namespace action {

Action::Action()
{
}

std::ostream& operator<<(std::ostream &stream, const Action& p)
{
	stream<<p.command();
    return stream;
}

} // namespace action
