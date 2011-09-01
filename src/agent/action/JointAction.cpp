/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: JointAction.cpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#include "JointAction.h"
#include "Template.hpp"
#include <sstream>
#include "robot/humanoid/Humanoid.h"
#include "core/WorldModel.h"

namespace action {

using namespace robot;
using namespace robot::humanoid;


JointAction::JointAction(bool b)
{
	mIsBodyJointAction=b;
	if(b)
	{
		fill(0); //TT: jid 2 to 21
	}
}


std::string JointAction::command() const
{
	std::stringstream ss;
    TJointMap::const_iterator jointEnd = mJointMap.end();

    FOR_EACH( iter, HUMANOID.getHingeJointEffectors() )
	{
		TJointMap::const_iterator jv = mJointMap.find(iter->first);
		if ( jv != jointEnd )
		{
        	ss<<'('<<iter->second<<' '<<jv->second<<')';
		}
    }

    FOR_EACH( iter, HUMANOID.getUniversalJointEffectors() )
	{
		unsigned int jid1 = iter->first;
		unsigned int jid2 = jid1+1;

		TJointMap::const_iterator jv1 = mJointMap.find(jid1);
		TJointMap::const_iterator jv2 = mJointMap.find(jid2);

		if ( jv1 != jointEnd )
		{
			if ( jv2 != jointEnd )
			{
        		ss<<'('<<iter->second<<' '<<jv1->second<<' '<<jv2->second<<')';
			}
			else
			{
				ss<<'('<<iter->second<<' '<<jv1->second<<" 0)";
			}
		}
		else if ( jv2 != jointEnd )
		{
			ss<<'('<<iter->second<<" 0 "<<jv2->second<<')';
		}
    }
	return ss.str();
}


void JointAction::fill(float v)
{
    FOR_EACH( iter, HUMANOID.getHingeJointEffectors() )
    {
        set(iter->first,v);
    }
    FOR_EACH( iter, HUMANOID.getUniversalJointEffectors() )
    {
		unsigned int jid1 = iter->first;
		unsigned int jid2 = jid1+1;
        set(jid1,v);
		set(jid2,v);
    }
}


void JointAction::setForCamera(unsigned int jid, math::AngDeg r)
{
	if(jid<=1)
	{
		mJointMap[jid]=math::deg2Rad(r);

		if(0==jid)
			WM.setSearchSpeedX(r);
		else //1==jid
			WM.setSearchSpeedY(r);
	}
}


} //end of namespace action

