/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "SoccerBot.h"

namespace robot { namespace humanoid {

        SoccerBot056::SoccerBot056()
            :Humanoid("rsg/agent/soccerbot056.rsg")
        {
            init();
        }

        SoccerBot056::~SoccerBot056()
        {
        }

        void SoccerBot056::init()
        {
            Humanoid::init();

            // sensor names
            // this can be set automatically
            mGyroRateNames.clear();
            mGyroRateNames["torso"] = 0;
            ///////////////add by allen 2010.3.15
            mAccelerometerNames.clear();
            mAccelerometerNames["torso"] = 0;
            ///////////////////////////////////////

        }

        SoccerBot058::SoccerBot058()
            :Humanoid("rsg/agent/soccerbot058/soccerbot.rsg")
        {
            init();
        }

        SoccerBot058::~SoccerBot058()
        {
        }

        void SoccerBot058::init()
        {
            Humanoid::init();

            // sensor names
            // this can be set automatically
            mGyroRateNames.clear();
            mGyroRateNames["torso"] = 0;
            ///////////////add by allen 2010.3.15
            mAccelerometerNames.clear();
            mAccelerometerNames["torso"] = 0;
            ///////////////////////////////////////
        }
        
    }} /* namespace robot::humanoid */
