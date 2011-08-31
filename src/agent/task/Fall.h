/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Fall.h
 *
 ****************************************************************************/
#ifndef TASK_FALL_H
#define TASK_FALL_H

#define ENABLE_TASK_FALL_LOG

#include "Task.h"
#ifdef ENABLE_TASK_FALL_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif
namespace task{
	enum Direction
	{
	RIGHT=0,
	LEFT,
        FRONT,
        BACK,
	UNKNOWN
	};
	
	class RightFall: public Task
	{
	public:
	RightFall(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();
	
	
	private:
	
	std::string mTaskName;
	
	};
	
	class LeftFall: public Task
	{
	public:
	LeftFall(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();
	
	
	private:
	
	std::string mTaskName;
	};
	
	class Fall: public Task
	{
	public:
	Fall();
	
	virtual ~Fall();
	
	Fall(Task* primary,Direction dir);
    
	virtual boost::shared_ptr<action::Action> perform();
	
	virtual bool isDone() const;
	 
	void SetDirection(Direction dir );
	
	private:
	
	Direction mDir;
	
	std::string mTaskName;
	
	};
	
	class KeepFall: public Task
	{
	public:
	KeepFall();
	
	virtual ~KeepFall();
	
	KeepFall(Task* primary,Direction dir);
    
	virtual boost::shared_ptr<action::Action> perform();
	
	virtual bool isDone() const;
	 
	void SetDirection(Direction dir );
	
	private:
	
	Direction mDir;
	
	};
	
}
 #endif

