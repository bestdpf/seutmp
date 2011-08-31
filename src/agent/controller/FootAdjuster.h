/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
 
#ifndef CONTROLLER_FOOT_ADJUSTER_H
#define CONTROLLER_FOOT_ADJUSTER_H

#define ENABLE_FOOT_ADJUST_LOG

#include "Singleton.hpp"
#include "../perception/Perception.h"
#include "../action/JointAction.h"
#include <boost/shared_ptr.hpp>
#ifdef ENABLE_FOOT_ADJUST_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace controller {
	
/** try to keep the foot horizontal */
class FootAdjuster: public Singleton<FootAdjuster>
{
public:
	FootAdjuster();
	~FootAdjuster(){}
	
	// foot meaning :
	// <0 : left foot
	// =0 : no foot
	// >0 : right foot
	void adjust(int foot, boost::shared_ptr<action::JointAction> act,
                bool adjustY = true);
		
protected:
	void adjustOneFoot(const perception::JointPerception& p,
                       const std::map<unsigned int, math::TransMatrixf>& m,
                       boost::shared_ptr<action::JointAction> act,
                       unsigned int shankID,
                       unsigned int jidX,
                       unsigned int jidY,
                       bool adjustY);

    void adjustKnee(const perception::JointPerception& p,
                    const std::map<unsigned int, math::TransMatrixf>& m,
                    boost::shared_ptr<action::JointAction> act,
                    unsigned int hipID,
                    unsigned int thighID1,
                    unsigned int thighID2,
                    unsigned int kneeID);
    

	DECLARE_GRAPHIC_LOGGER;
};

#define FOOT_ADJUSTER controller::FootAdjuster::GetSingleton()
 
} // namespace controller

#endif // CONTROLLER_FOOT_ADJUSTER_H
