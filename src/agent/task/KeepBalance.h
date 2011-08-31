/***************************************************************************
 *            KeepBalace.h
 *
 *  Thu Oct 18 11:53:58 2007
 *  Copyright  2007  User
 *  Email
 ****************************************************************************/


#ifndef TASK_KEEPBALANCE_H
#define TASK_KEEPBALANCE_H

#define ENABLE_TASK_KEEPBALANCE_LOG

#include "Singleton.hpp"
#include "Task.h"
#include "GetUp.h"
#ifdef ENABLE_TASK_KEEPBALANCE_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif


#define IWANTTOLIE	0
#define IWANTTODIVE	1




namespace task{

using namespace boost;
using namespace action;

class KeepBalance: public Task
{
public:

	KeepBalance();

	virtual ~KeepBalance();

	virtual shared_ptr<action::Action> perform();

	enum KeepBalanceState
	{
		LIED_STATE = 0,				//lied state
		LYING_STATE,				//lying state
		DIVED_STATE,				//dived state
		DIVING_STATE,				//diving state
		//LRROLLED_STATE,				//×óÓÒÇãµ¹µÄ×´Ì¬
		LEFTFALL_STATE,
		RIGHTFALL_STATE,
		BALANCE_STATE				//Æ½ºâ×´Ì¬
	};

	typedef KeepBalanceState KBS;

    void reset();

private:



	void analysisWhatToDo();

private:


	static KBS mCurrentState;

	static KBS mPossibleState;

	static KBS mLastPossibleState;

	static float mStateKeepingStartTime;

	static float unBalanceTime;

	bool	lieordive;

	DECLARE_STATIC_GRAPHIC_LOGGER;

public:
	void updateState();
	void	setLieOrDive(bool n)	{lieordive=n;}
	bool	getLieOrDive()	{return	lieordive;}
	KBS  getCurrentState() {return mCurrentState;}
};

} // namespace task

#endif // TASK_KEEPBALANCE_H

