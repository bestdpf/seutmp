/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_HUMANOID_NAO_H_
#define _ROBOT_HUMANOID_NAO_H_

#include "Humanoid.h"

namespace robot { namespace humanoid{

        class Nao : public Humanoid
        {
        public:

            Nao();

            virtual ~Nao();

            virtual bool legInverseKinematics(bool isLeft,
                                              const TransMatrixf& torsoMat,
                                              const TransMatrixf& footMat,
                                              std::map<unsigned int, math::AngDeg>& angles) const;
            
            bool legIK(bool isLeft,
                       const TransMatrixf& torsoMat,
                       const TransMatrixf& footMat,
                       std::map<unsigned int, math::AngDeg>& angles,
                       int count) const;

            bool legIK2(bool isLeft,
                       const TransMatrixf& torsoMat,
                       const TransMatrixf& footMat,
                       std::map<unsigned int, math::AngDeg>& angles) const;

            bool inverseHip(boost::shared_ptr<const Bone> hip1,
                            boost::shared_ptr<const Bone> hip2,
                            boost::shared_ptr<const Bone> thigh,
                            const TransMatrixf& torsoMat,
                            const TransMatrixf& thighMat,
                            std::map<unsigned int, math::AngDeg>& angles) const;

            // joint ids
            static const int JID_NECK = 0;
            static const int JID_HEAD = 1;
            static const int JID_RSHOULDER = 2;
            static const int JID_RUPPERARM = 3;
            static const int JID_RELBOW = 4;
            static const int JID_LSHOULDER = 5;
            static const int JID_LUPPERARM = 6;
            static const int JID_LELBOW = 7;
            static const int JID_RHIP1 = 8;
            static const int JID_RHIP2 = 9;
            static const int JID_RTHIGH = 10;
            static const int JID_RSHANK = 11;
            static const int JID_RANKLE = 12;
            static const int JID_RFOOT = 13;
            static const int JID_LHIP1 = 14;
            static const int JID_LHIP2 = 15;
            static const int JID_LTHIGH = 16;
            static const int JID_LSHANK = 17;
            static const int JID_LANKLE = 18;
            static const int JID_LFOOT = 19;
            
        protected:
            virtual void init();

        private:
            float mThighLength;
            float mShankLength;
            float mMaxLegLength;
        };
        
    } } /* namespace robot::humanoid */



#endif /* _ROBOT_HUMANOID_NAO_H_ */
