/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef PERCEPTION_BASIC_PERCEPTION_H
#define PERCEPTION_BASIC_PERCEPTION_H

#include "parser/SexpParser.hpp"

namespace perception {

#define SS_NAME_STR "n"
    
/** @class abstract class of perception
 */
class BasicPerception
{
public:
	BasicPerception(){}
	virtual ~BasicPerception(){}
	
	virtual bool update(const sexp_t* sexp) = 0;
};

} // namespace perception

#endif // PERCEPTION_BASIC_PERCEPTION_H
