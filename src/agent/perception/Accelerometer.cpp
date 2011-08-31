/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Accelerometer.cpp,v 1.0 2010/03/15  Allen Exp $
 *
 ****************************************************************************/

#include "Accelerometer.h"
#include "robot/humanoid/Humanoid.h"

namespace perception {

Accelerometer::Accelerometer()
{
}

Accelerometer::~Accelerometer()
{
}

/* (GYR (name torsogyro) (-0.12 -0.01 -0.89))  */
bool Accelerometer::update(const sexp_t* sexp)
{
	// get the name
	std::string name;
	if ( !parser::SexpParser::parseGivenValue(sexp,SS_NAME_STR,name) )
	{
		std::cerr<<"can not get the ACC name!\n";
		return false;
	}

	// get the id
    int id = HUMANOID.getAccelerometerId(name);
	if ( id < 0 )
	{
		std::cerr<<"unknown ACC name: "<<name<<'\n';
		return false;
	}

	// get the rt
	sexp = sexp->next;
    math::Vector3f rate;
    if ( !parser::SexpParser::parseGivenValue(sexp,"a",rate) )
	{
		std::cerr<<"can not get ACC a value\n";
		return false;
	}

	// set the rt
	mRates[id] = rate;
	return true;
}

std::ostream& operator<<(std::ostream &stream, const Accelerometer& g)
{
	stream<<"(ACC ";
	for( Accelerometer::TRateMap::const_iterator iter = g.mRates.begin(); iter!=g.mRates.end(); ++iter )
	{
		stream<<"(name "<<HUMANOID.getAccelerometerName(iter->first)<<")("<<iter->second<<")";
	}
	stream<<")";
	return stream;
}

} // namespace perception
