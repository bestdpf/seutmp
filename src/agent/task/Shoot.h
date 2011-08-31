/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef TASK_SHOOT_H
#define TASK_SHOOT_H

#define ENABLE_TASK_SHOOT_LOG

#include "BasicKick.h"
#ifdef ENABLE_TASK_SHOOT_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{

using namespace std;

class Shoot: public BasicKick
{
public:
	Shoot( const math::Vector2f& target, bool isLeftLeg = false, bool maxForce=false, Task* primary = NULL );

	virtual boost::shared_ptr<action::Action> perform();

private:

	/*
	* 生成踢球脚ankle角度到球运动距离的map
	*/
	virtual void generateAnkleDistTable();

	virtual void generateDurationDistTable();

	virtual AngDeg getDesiredAnkleAngle();

	virtual float getDesiredDuration();

private:
	float mAcc2Duration;
	bool mMaxForce;

	///////////////////////////////////// TT test
	bool mIsFirstTime;
	////////////////////

	DECLARE_STATIC_GRAPHIC_LOGGER;
};


}

#endif // TASK_SHOOT_H

