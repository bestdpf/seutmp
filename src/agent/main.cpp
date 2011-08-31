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
 * @file   main.cpp
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Fri Mar 21 21:47:50 2008
 *
 * @brief  the main entrance of the seu-spark-agent program
 *
 *
 */

#include "configuration/Configuration.h"
#include "soccer/TeamPlayer.h"


using namespace std;


int main(int argc, char* argv[])
{
    try{
        // parse the input command
        CONF.init(argc, argv);

        // initialize the agent: connect to server, create the agent, beam, etc.
        if ( !AGENT.init() ){
            return 1;
        }

        // the agent 'sense - think - act' main loop
        //AGENT.run();
        AGENT.runMultiThreads();
        //test::TestWalk tw;
        //tw.runLoopWithoutDelay();
        //tw.runLoopWithDelay();

        // disconnect, save something ...
        AGENT.done();

        return 0;
    }
    catch ( ClassException<configuration::Formation>& e )
    {
        cerr<<e.what()<<endl;
    }
}
