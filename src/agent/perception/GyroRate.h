/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: GyroRate.h,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#ifndef PERCEPTION_GYRO_RATE_H
#define PERCEPTION_GYRO_RATE_H

#include "math/Math.hpp"
#include "BasicPerception.h"


namespace perception {

class GyroRate: public BasicPerception
{
public:
	GyroRate();
	~GyroRate();
	
	virtual bool update(const sexp_t* sexp);

	friend std::ostream& operator<<(std::ostream &stream, const GyroRate& g);

	typedef std::map<unsigned int, math::Vector3f> TRateMap;

    const math::Vector3f& rate(unsigned int id) const 
        {
            return mRates.find(id)->second;
        }
    
    
private:
	TRateMap mRates;
};

std::ostream& operator<<(std::ostream &stream, const GyroRate& g);

} // namespace perception

#endif // PERCEPTION_GYRO_RATE_H
