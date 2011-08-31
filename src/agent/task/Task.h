/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

/**
 * @file   Task.h
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Sun Oct  7 00:46:40 2007
 *
 * @brief  Abstract class of task based on time order
 *
 *
 */

#ifndef TASK_TASK_H
#define TASK_TASK_H

#include "../action/Action.h"
#include <list>
#include <boost/shared_ptr.hpp>

namespace task{

class Task
{
public:
	/**
	 * create a task
	 *
	 * @param time the whole duration time of this task
	 * @param primary the primary task, if NULL means the primariest task
	 */
	Task(float time, Task* primary);

	virtual ~Task(){}

	/**
	 * if the current state is suitable to achieve this task
	 *
	 *
	 * @return boolen indicates whether the task is achieveable
	 */
	virtual bool isAchieveable() const;

	/**
	 * if the task can be terminnated in the current state
	 *
	 *
	 * @return boolen indicates whethter the task is terminable
	 */
	virtual bool isTerminable() const;

	/**
	 * indicates the task is done or not,
	 * i.e. the target is satisfied
	 *
	 * @return boolen indicate the task is done or not
	 */
	virtual bool isDone() const;

	/**
	 * perform the task forward one simulation cycle,
	 * i.e. make the action according to current state and target
	 *
	 * @return the action samrt point which can executed by server
	 */
	virtual boost::shared_ptr<action::Action> perform();

	/**
	 * revise the task by another task
	 *
	 * @param rt the other task
	 *
	 * @return if the task if revised
	 */
	virtual bool revise( boost::shared_ptr<const Task> /*rt*/ )
		{ return false; }

	/**
	 * give an change to dervied class to update the sub task list
	 */
	virtual void updateSubTaskList();

	/**
	 * try to stop the task
	 * if the sub task is not achieveable --> clear it
	 * if the sub task is terminable --> clear it
	 *
	 * @return if all sub tasks are cleared
	 */
	bool stop();

	/**
	 * if the task is the top parent task
	 * i.e. mParentTask == NULL
	 *
	 * @return
	 */
	bool isTop() const
		{
			return NULL == mParentTask;
		}

	/**
	 * @return if the remain time is negative
	 */
	bool isTimeOut() const
		{
			return getRemainTime() < 0.001f;								/////terrymimi-note change 0.005 to 0.001
		}

	/**
	 * @return if the first task ( which in the front of
	 * list or its self ) is done
	 */
	bool isSubDone() const;

	/**
	 * @return get the pointer to the first sub task
	 */
	boost::shared_ptr<Task> getFirstSubTask();
	boost::shared_ptr<const Task> getFirstSubTask() const;

	/**
	 * @return get the pointer to the second sub task
	 */
	boost::shared_ptr<Task> getSecondSubTask();
	boost::shared_ptr<const Task> getSecondSubTask() const;

	/**
	 * @return get the first lowest sub task
	 */
	boost::shared_ptr<const Task> getFirstOfAllSubTask() const;

	/**
	 * get the next sub task, then the first sub task can ajust itself
	 *
	 *
	 * @return the const pointer of the next sub task
	 */
	boost::shared_ptr<const Task> getNextOfAllSubTask() const;

	/**
	 * @return get the remain time of this task
	 */
	float getRemainTime() const;

	/**
	 * @return if there is only one sub-task, meaning need to append taskes
	 */
	bool isSubTaskOfAllLessThanTwo() const;

	bool isSubTaskOfAllLessThanThree() const;								/////terrymimi

	/**
	 * append the task t to the sub task list
	 */
	void append(boost::shared_ptr<Task> t)
		{
			mSubTaskList.push_back(t);
		}

	/**
	* @return true if this action is first used
	*/
	bool isFirstDoAction() const;

	/**
	 * clear the sub tasks directly
	 *
	 */
	void clear() { mSubTaskList.clear(); }

	//////////////////////////////////////////// TT test
	int getSubTaskListSize()
	{
		return mSubTaskList.size();
	}
	///////////////////////////////////////////

protected:
	/// the avaiable time before the task complete
	float mDuration;
	float mStartTime;

	/// point to parent task
	Task* mParentTask;

	/// the sub-tasks' list
	typedef std::list< boost::shared_ptr<Task> > TTaskList;
	TTaskList mSubTaskList;
};


} //end of namespace task

#endif // TASK_TASK_H

