/***************************************************************************
 *            GetUp.h
 *
 *  Thu Oct 18 13:18:38 2007
 *  Copyright  2007  User
 *  Email
 ****************************************************************************/

#ifndef TASK_GETUP_H
#define TASK_GETUP_H

#define ENABLE_TASK_GETUP_LOG

#include "Task.h"
#ifdef ENABLE_TASK_GETUP_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{

class PushUpDirectly: public Task
{
public:
	PushUpDirectly(Task* primary);

	static bool shouldDo ( int currentState, int possibleState );

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDone() const { return isTimeOut(); }

	virtual bool isTerminable() const { return false; }

private:
	bool isNotReady();		

	std::string mTaskName;

	float bufferTime;

	DECLARE_STATIC_GRAPHIC_LOGGER;
};


class PushUpPre: public Task
{
public:
	PushUpPre(int currentState, int possibleState, Task* primary);

	static bool shouldDo ( int currentState, int possibleState );

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDone() const { return isTimeOut(); }

	virtual bool isTerminable() const { return mTerminable; }

private:
	bool isNotReady();		

	std::string taskName;

	bool mTerminable;
};


class MantodeaUpDirectly: public Task
{
public:
	MantodeaUpDirectly(Task* primary);

	static bool shouldDo ( int currentState, int possibleState );

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDone() const { return isTimeOut(); }

	virtual bool isTerminable() const { return false; }

private:
	std::string mTaskName;

	float bufferTime;
};

class MantodeaUpPre: public Task
{
public:
	MantodeaUpPre(int currentState, int possibleState, Task* primary);

	static bool shouldDo ( int currentState, int possibleState );

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDone() const { return isTimeOut(); }

	virtual bool isTerminable() const { return mTerminable; }

private:
	std::string taskName;

	bool mTerminable;
};

class Roll: public Task
{
public:
	Roll(Task* primary);

	static bool shouldDo ( int currentState, int possibleState );

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDone() const { return isTimeOut(); }

	virtual bool isTerminable() const { return false; }

private:
};

class ShakeBody: public Task
{
public:
	ShakeBody(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDone() const { return isTimeOut(); }

	virtual bool isTerminable() const { return false; }

private:
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class GetUpFromLie: public Task
{
public:
	GetUpFromLie(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDoneByTime() const {return isTimeOut();}

	virtual bool isDoneByAng()	const;

	virtual bool isTerminable() const;

	virtual bool isGetUpFailed() const;

	virtual bool isDone() const {return isDoneByTime();}

private:
	bool mTerminable;

	std::string mTaskName;

	DECLARE_STATIC_GRAPHIC_LOGGER;
};



class GetUpFromDive: public Task
{
public:
	GetUpFromDive(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDoneByTime() const {return isTimeOut();}

	virtual bool isDoneByAng()	const;

	virtual bool isTerminable() const ;

	virtual bool isGetUpFailed() const;

	virtual bool isDone() const {return isDoneByTime();}

private:
	bool mTerminable;

	std::string mTaskName;

	DECLARE_STATIC_GRAPHIC_LOGGER;

};


class LeftFallToLie: public Task
{
public:
	LeftFallToLie(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDoneByTime() const {return isTimeOut();}

	virtual bool isDoneByAng()	const;

	virtual bool isTerminable() const {return mTerminable;}

	//virtual bool isFailed() const;

	virtual bool isDone() const {return isDoneByTime();}

private:
	bool mTerminable;

	std::string mTaskName;

};


class RightFallToLie: public Task
{
public:
	RightFallToLie(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDoneByTime() const {return isTimeOut();}

	virtual bool isDoneByAng()	const;

	virtual bool isTerminable() const {return mTerminable;}

	//virtual bool isFailed() const;

	virtual bool isDone() const {return isDoneByTime();}

private:
	bool mTerminable;

	std::string mTaskName;

};


class RightFallToDive: public Task
{
public:
	RightFallToDive(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDoneByTime() const {return isTimeOut();}

	virtual bool isDoneByAng()	const;

	virtual bool isTerminable() const {return mTerminable;}

	//virtual bool isFailed() const;

	virtual bool isDone() const {return isDoneByTime();}

private:
	bool mTerminable;

	std::string mTaskName;

};




class LeftFallToDive: public Task
{
public:
	LeftFallToDive(Task* primary);

	virtual boost::shared_ptr<action::Action> perform();

	virtual bool isDoneByTime() const {return isTimeOut();}

	virtual bool isDoneByAng()	const;

	virtual bool isTerminable() const {return mTerminable;}

	//virtual bool isFailed() const;

	virtual bool isDone() const {return isDoneByTime();}

private:
	bool mTerminable;

	std::string mTaskName;

};


}

#endif		//TASK_GETUP_H


