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
 * \file            Agent.h
 *
 * \brief agent基类/the basic agent class.
 *
 *
 * 实现基本的agent: 初始化,感知-思考-动作的主循环
 *
 */

#ifndef CORE_AGENT_H
#define CORE_AGENT_H

#include <fstream>
#include <boost/thread/thread.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>
#include "action/Action.h"
#include "perception/Perception.h"


namespace core {

class Agent
{
public:
    Agent();

    virtual ~Agent();

    /** initalization the agent */
    virtual bool init();

    /** the main sense - think - act main loop */
    void run();

    /** 
     * the sense - think - act loop in multi threads
     */
    void runMultiThreads();
    
    /** do something before exit */
    virtual void done();
    
    /** think what need to do, i.e make the decision */
    virtual boost::shared_ptr<action::Action> think() = 0;

    /** 
     * get the sense from CommUnit
     * 
     * @return the perception generated from the message
     */
    boost::shared_ptr<perception::Perception> sense();

    /** 
     * execute the action
     * 
     * @param a the action which will be executed
     */
    void perform( boost::shared_ptr<action::Action> a );

    /** 
     * The thread which manage the sense loop
     * It try to get the message from the server ASAP
     */
    void senseThread();

    /** 
     * The thread which manage the think loop
     * It allows the think loop more than one cycle, i.e. 20ms
     */
    void thinkThread();

    /** 
     * The thread which manage the act loop
     * It try to send the action before the server cycle ends
     */
    void actThread();

    /** 
     * calculate the next act time according to the last sense time.
     * i.e. the next act time should not be later after server ends,
     * and also, if missing some messages, the time interval should
     * be suited to the possible cycles
     */
    void calculateNextActTime();

    boost::shared_ptr<const action::Action> getLastAction()
        {
            boost::mutex::scoped_lock lock(mLastActMutex);
            boost::shared_ptr<const action::Action> act = mLastAction;
            return act;
        }
    
private:
    /// cache the last perception
    boost::shared_ptr<perception::Perception> mPer;

    /// the action which will be executed
    boost::shared_ptr<action::Action> mAct;

    /// the action last performed
    boost::shared_ptr<action::Action> mLastAction;

    /// the last system time when the send action to the server
    boost::xtime mLastActTime;
    boost::xtime mLastSenseTime;
    boost::xtime mNextActTime;
    
    /// the mutex of the last perception
    boost::mutex mPerMutex;
    /// the condition of the last perception
    boost::condition mPerCond;

    boost::mutex mTimeMutex;
    boost::condition mTimeCond;

    boost::mutex mActMutex;
    boost::mutex mLastActMutex;
    
    /// is alive? i.e. connect to the server
    bool mIsAlive;

    /// logging all the message from the server
    std::ofstream mSenseLog;
};

} // namespace core

#endif // CORE_AGENT_H
