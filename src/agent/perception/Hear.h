/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef _PERCEPTION_HEAR_H_
#define _PERCEPTION_HEAR_H_

#include "BasicPerception.h"

namespace perception{

    class Hear : public BasicPerception
    {
    public:
        Hear(){}

        virtual ~Hear(){}

        virtual bool update(const sexp_t* sexp);

        float time() const
            {
                return mTime;
            }

        bool isSelfMsg() const 
            {
                return mSelf;
            }

        math::AngDeg direction() const
            {
                return mDirection;
            }

        const std::string& message() const
            {
                return mMsg;
            }
        
    private:
        float mTime;
        bool mSelf;
        math::AngDeg mDirection;
        std::string mMsg;
    };
    
} // namespace perception

#endif /* _PERCEPTION_HEAR_H_ */
