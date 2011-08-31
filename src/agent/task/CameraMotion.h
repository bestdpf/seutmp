/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: CameraMotion.h
 *
 ****************************************************************************/

#ifndef _CAMERAMOTION_H
#define	_CAMERAMOTION_H

#define ENABLE_TASK_CAMERA_MOTION_LOG

#ifdef ENABLE_TASK_CAMERA_MOTION_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

#include "Task.h"
#include "action/JointAction.h"
#include "perception/Vision.h"
#include "controller/Timing.h"
#include "configuration/Configuration.h"
#include "robot/humanoid/Nao.h"
#include "action/Actions.h"
namespace task {
using namespace boost;
    using namespace action;
    using namespace perception;
    using namespace std;
    using namespace controller;
    using namespace robot::humanoid;
    using namespace serversetting;


    class CameraMotion : public Task {
public:
	CameraMotion(int mode=-1, Task* primary=NULL);

	virtual ~CameraMotion();

	virtual boost::shared_ptr<action::Action> perform();

	boost::shared_ptr<action::JointAction> searchBall();

	boost::shared_ptr<action::JointAction> searchBallAfterGreat();

	boost::shared_ptr<action::JointAction> stickToBall();

	boost::shared_ptr<action::JointAction> searchFlags();

	boost::shared_ptr<action::JointAction> dontTurn();

	boost::shared_ptr<action::JointAction> forAttacking();

	boost::shared_ptr<action::JointAction> forTest();


private:
	DECLARE_STATIC_GRAPHIC_LOGGER;

	int mMode;

};

}

#endif	/* _CAMERAMOTION_H */
