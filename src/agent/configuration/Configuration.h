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
 * @file   Configuration.h
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Fri Mar 21 21:37:12 2008
 * 
 * @brief  the class Configuration is the collection of seu-spark agent's configurations
 * 
 * 
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "Singleton.hpp"
#include "Options.h"
#include "Formation.h"

namespace configuration
{
    class Configuration: public Singleton<Configuration>
    {
    public:
        /** 
         * the construction of Configuration, prints the greetings
         * 
         */
        Configuration();
        
        /** 
         * initilize the configuration according the program options
         * 
         * @param argc the arguments' number
         * @param argv the arguments' value
         */
        void init(int argc, char* argv[]);

        /** 
         * get the options
         * @return the refernece of Options
         */
        const Options& options() const
            {
                return mOpt;
            }

        /** 
         * get the formation
         * @return the reference of formation
         */
        const Formation& formation() const
            {
                return mFormation;
            }
        
    private:
        /// program options
        Options mOpt;

        /// the formation
        Formation mFormation;
    };
}

#define CONF configuration::Configuration::GetSingleton()
#define OPTS configuration::Configuration::GetSingleton().options()
#define FM configuration::Configuration::GetSingleton().formation()

#endif // CONFIGURATION_H

