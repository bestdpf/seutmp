/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Accelerometer.h,v 1.0 2010/03/15  Allen Exp $
 *
 ****************************************************************************/

#ifndef PERCEPTION_ACCELEROMETER_H
#define	PERCEPTION_ACCELEROMETER_H

#include "math/Math.hpp"
#include "BasicPerception.h"


namespace perception {

    class Accelerometer : public BasicPerception
    {
    public:
        Accelerometer();
        ~Accelerometer();

        virtual bool update(const sexp_t* sexp);
        friend std::ostream & operator<<(std::ostream &stream, const Accelerometer& g);
        typedef std::map<unsigned int, math::Vector3f> TRateMap;

        const math::Vector3f& rate(unsigned int id) const {
            return mRates.find(id)->second;
        }

    private:
        TRateMap mRates;

    };
    std::ostream & operator<<(std::ostream &stream, const Accelerometer& g);

} // namespace perception

#endif // PERCEPTION_ACCELEROMETER_H

