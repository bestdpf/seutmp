/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Action.h,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#ifndef ACTION_ACTION_H
#define ACTION_ACTION_H

#include <string>

namespace action {

class Action
{
public:

    Action();
	
	virtual ~Action(){};
	
	virtual std::string command() const = 0;
	
    friend std::ostream& operator<<(std::ostream &stream, const Action& p);
    
};

std::ostream& operator<<(std::ostream &stream, const Action& p);

} // namespace action

#endif // ACTION_ACTION_H
