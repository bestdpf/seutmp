/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Proportion.h"

namespace controller {

boost::shared_ptr<action::Action>
Proportion::control(const perception::JointPerception& p0, const perception::JointPerception& p1)
{
	boost::shared_ptr<action::JointAction> act(new action::JointAction);
	
	// hinge joints
	typedef perception::JointPerception::TJointMap TJointMap;
	const TJointMap& hinge0 = p0.jointMap();
	const TJointMap& hinge1 = p1.jointMap();

    TJointMap::const_iterator end0 = hinge0.end();
	for( TJointMap::const_iterator iter=hinge1.begin(); iter!=hinge1.end(); ++iter )
    {
        unsigned int jid = iter->first;
        TJointMap::const_iterator iter0 = hinge0.find(jid);
        if ( iter0 == end0 ) continue;
        /// desired angle
        math::AngDeg angD = iter->second.angle();//SS.restricJointAngle(jid, iter->second.angle());
        /// current angle
        math::AngDeg angC = iter0->second.angle();
        math::AngDeg ang = math::calClipAng(angD, angC);
        
		ang *= mK;
		act->set(jid,ang);
    }

	return boost::shared_static_cast< action::Action >(act);
}
	
} // namespace controller
