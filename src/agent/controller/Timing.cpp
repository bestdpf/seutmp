/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Timing.cpp,v 1.1 2007/03/13 07:39:26 xy Exp $
 *
 ****************************************************************************/

#include "Timing.h"
#include "math/Math.hpp"
#include "robot/humanoid/Humanoid.h"

namespace controller {

using namespace std;
using namespace boost;
using namespace math;
using namespace serversetting;
using namespace action;
using namespace perception;


Timing::Timing()
{
}


shared_ptr<Action> Timing::control(const Perception& p,
                                   const JointPerception& p1,
								   float deltaTime)
{
    return control(p, p1.jointAngles(), deltaTime);
}


shared_ptr<Action> Timing::control(const Perception& predictedPerception,
                                   const map<unsigned int,math::AngDeg>& desiredJointAngleMap,
								   float deltaTime)
{
	shared_ptr<JointAction> jact(new JointAction);

	deltaTime= max(0.04f,deltaTime); //0.1f
	float invDeltaTime= 1/deltaTime;

	const TJointMap& jointMap= predictedPerception.joints().jointMap(); //map<uint,Joint>
	TJointMap::const_iterator jointMapEnd= jointMap.end();

	FOR_EACH(iter,desiredJointAngleMap)
	{
		unsigned int jid= iter->first;
		TJointMap::const_iterator jointFind= jointMap.find(jid);
		if(jointFind==jointMapEnd) continue;

		//we restrict the desired joint angle here
		AngDeg desiredAng= iter->second;
		AngDeg currentAng= jointFind->second.angle();
		AngDeg ang= HUMANOID.calJointTurnAngle(jid, currentAng, desiredAng);
		float v=ang*invDeltaTime;
		jact->set(jid,v); //TT: you need to know that it sets [turning-speed] of joints (jid: 2~21)
	}

	return jact;
}


shared_ptr<action::Action> Timing::control(const JointPerception& p0,
										   const JointPerception& pf,
										   float tf,
										   const JointPerception& pff,
										   float tff)
{
	return control(p0, pf.jointAngles(), tf, pff.jointAngles(), tff);
}


shared_ptr<action::Action> Timing::control(const perception::JointPerception& p0,
                                           const std::map<unsigned int, math::AngDeg>& pf,
										   float tf,
                                           const std::map<unsigned int, math::AngDeg>& pff,
										   float tff)
{
    // set the min tf to sim_step
    tf = max(sim_step,tf)+sim_step;
	float tfInv = 1.0f / tf; // 1/tf
	float tf2Inv = tfInv * tfInv; // 1/(tf^2)
	float tf3Inv = tfInv * tf2Inv; // 1/(tf^3)
	float tffInv = 1.0f / tff; // 1/tff
	float t = sim_step+sim_step;
	float t2 = t*t;
	shared_ptr<JointAction> act(new JointAction);

	// joints
	typedef JointPerception::TJointMap TJointMap;
	const TJointMap& j0 = p0.jointMap();

	TJointMap::const_iterator end0 = j0.end();
	map<unsigned int, AngDeg>::const_iterator endff = pff.end();
	FOR_EACH(iterf, pf){
		unsigned int jid = iterf->first;
		TJointMap::const_iterator iter0 = j0.find(jid);
		if ( end0 == iter0 ) continue; // no perception of this joint!
		AngDeg rt0 = iter0->second.rate();
		AngDeg ax0 = iter0->second.angle();

		/// we may restrict the desired joint angle here
		AngDeg axf = iterf->second;
		AngDeg ax0f = HUMANOID.calJointTurnAngle(jid, ax0, axf);
		AngDeg rtf = 0;

		map<unsigned int, AngDeg>::const_iterator iterff = pff.find(jid);
		if ( tff > 0 && endff != iterff ){
			// calculate the rtf according axf and axff
			AngDeg axff = iterff->second;
			AngDeg axfff = HUMANOID.calJointTurnAngle(jid, axf, axff);
			if ( axfff * ax0f > 0 ){
				// the same direction
				rtf = ( ax0f * tfInv + axfff * tffInv ) * 0.5f;
			}
		}

		// if (rt0*ax0f<0)
		//     rt0 = ax0f * tfInv;

		AngDeg rt = rt0 + ( 3*ax0f*tf2Inv - ( 2*rt0 + rtf )* tfInv )*t
			+ ( -2*(ax0f)*tf3Inv + ( rt0 + rtf )*tf2Inv )*t2;

		if ( rt*ax0f<0 ){
			rt = ax0f * tfInv;
		}

		act->set(jid,rt);
	}

	return act;
}


//use useTime to set joint jid to angle
void Timing::controlJointAngle(shared_ptr<Action> act,
							   unsigned int jid,
							   AngDeg ang0,
							   AngDeg ang1,
							   float useTime)
{
	shared_ptr<JointAction> jact = shared_dynamic_cast<JointAction>(act);
	if ( NULL == jact.get() ) return; // not a joint action

	ang0 = normalizeAngle(ang0);
	ang1 = normalizeAngle(ang1);
	AngDeg angSpeed = normalizeAngle(ang1 - ang0)/useTime;
	angSpeed = clamp(angSpeed,-8900.0f,8900.0f);
	jact->set(jid,angSpeed);
}


void Timing::controlJointToAngleY(shared_ptr<Action> act,
								  unsigned int jid,
								  const TransMatrixf& mat,
								  AngDeg angle,
								  float useTime)
{
	AngDeg angNow
		= atan2Deg(mat.o()[2],sqrt(pow2(mat.o()[0]) + pow2(mat.o()[1])));

	controlJointAngle(act, jid, angNow, angle, useTime);
}


} //end of namespace controller

