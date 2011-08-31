/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Timing.h,v 1.1 2007/03/13 07:39:26 xy Exp $
 *
 ****************************************************************************/

#ifndef CONTROLLER_TIMING_H
#define CONTROLLER_TIMING_H

#include <boost/shared_ptr.hpp>
#include "../action/JointAction.h"
#include "../perception/Perception.h"

namespace controller {

class Timing
{
public:

	~Timing(){}

	static boost::shared_ptr<action::Action>
	control(const perception::Perception& p0, const perception::JointPerception& p1, float deltaTime);

    static boost::shared_ptr<action::Action>
	control(const perception::Perception& p0, const std::map<unsigned int, math::AngDeg>& p1, float deltaTime);

    /**
     * The trajectory of each joint is expressed as a cubic polynomial,
     * to make the motion of robot smooth.
     *
     * @param p0 current joint angle and velocity
     * @param pf desired joint angle
     * @param tf the duration for reach desired time
     * @param pff the next desired joint angle,
     * used for determining veocity at tf
     * @param tff the duration between pf and pff
     *
     * @return the action of current state
     */
    static boost::shared_ptr<action::Action>
    control(const perception::JointPerception& p0,
            const perception::JointPerception& pf, float tf,
            const perception::JointPerception& pff, float tff);

    static boost::shared_ptr<action::Action>
    control(const perception::JointPerception& p0,
            const std::map<unsigned int, math::AngDeg>& pf, float tf,
            const std::map<unsigned int, math::AngDeg>& pff, float tff);

    /**
     * let this jid to the angle
     */
	static void controlJointAngle(boost::shared_ptr<action::Action> act,
                                  unsigned int jid,
                                  math::AngDeg ang0,
                                  math::AngDeg ang1,
                                  float useTime=serversetting::sim_step);

    /**
     * 设置这个jid相对于自己坐标系y，也就是全局坐标系中x的夹角为angle
     */
    static void controlJointToAngleY(boost::shared_ptr<action::Action> act,
                                     unsigned int jid,
                                     const math::TransMatrixf& mat,
                                     math::AngDeg angle,
                                     float useTime=serversetting::sim_step);

private:
	Timing(); // do not try to create an instance

	typedef std::map<unsigned int, perception::Joint> TJointMap;

};

} // namespace controller

#endif // CONTROLLER_TIMING_H
