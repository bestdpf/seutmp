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
 * @file   Configuration.cpp
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Fri Mar 21 21:41:00 2008
 * 
 * @brief  the class Configuration is the collection of seu-spark agent's configurations
 * 
 * 
 */

#include "Configuration.h"
#include "robot/humanoid/SoccerBot.h"
#include "robot/humanoid/Nao.h"
namespace configuration{

    Configuration::Configuration()
    {
        // print greetings message
        mOpt.greetings();
    }
    
    void Configuration::init(int argc, char* argv[])
    {
        mOpt.parseCmdLine(argc,argv);

        // load the formation_conf file
        mFormation.loadFile(mOpt.arg<std::string>("formation_conf"));
        mFormation.setMyFormation("2v2",mOpt.arg<unsigned int>("unum"));
        // assemble the robot
        const std::string& robotName = mFormation.getMy().robot;
        if ( "rsg/agent/soccerbot056.rsg" == robotName ){
            ROBOT.init<robot::humanoid::SoccerBot056>();
        }
        else if ( "rsg/agent/soccerbot058/soccerbot.rsg" == robotName ){
            ROBOT.init<robot::humanoid::SoccerBot058>();
        }
        else if ( "rsg/agent/nao/nao.rsg" == robotName ){
            ROBOT.init<robot::humanoid::Nao>();
        }
        else{
            throw ClassException<Configuration>("The robot "+robotName+
                "is not support!");
        }
    }
    
} // namespace configuration
