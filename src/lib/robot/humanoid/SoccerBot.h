/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef _ROBOT_HUMANOID_SOCCERBOT_H_
#define _ROBOT_HUMANOID_SOCCERBOT_H_

#include "Humanoid.h"

namespace robot { namespace humanoid{

        /* the class for soccerbot056 */
        class SoccerBot056 : public Humanoid
        {
        public:
            
            SoccerBot056();

            virtual ~SoccerBot056();

        protected:
            virtual void init();

        private:
            //map<EBone,shared_ptr<Bone> > mBoneMap;
        };

        class SoccerBot058 : public Humanoid
        {
        public:
            SoccerBot058();

            virtual ~SoccerBot058();

        protected:
            virtual void init();
        };
        
    }} /* namespace robot:: humanoid */

#endif /* _ROBOT_HUMANOID_SOCCERBOT_H_ */
