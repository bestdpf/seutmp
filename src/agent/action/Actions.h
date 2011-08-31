/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef ACTION_ACTIONS_H
#define ACTION_ACTIONS_H

/**
 * @file   Actions.h
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Wed Sep 19 23:31:32 2007
 *
 * @brief  sort some actions in one `action'
 *
 * such as beam and joint, they can be performed at the same time
 */


#include "Action.h"
#include <vector>
#include <boost/shared_ptr.hpp>

namespace action {

class Actions : public Action
{
public:
	typedef std::vector< boost::shared_ptr<Action> > TActionPtrs;

	Actions();

	virtual ~Actions();

	virtual std::string command() const;

	void add( boost::shared_ptr<Action> act );

	const TActionPtrs& get() const
	{
		return mActs;
	}

private:
	TActionPtrs mActs;
};


} //end of namespace action

#endif // ACTION_ACTIONS_H

