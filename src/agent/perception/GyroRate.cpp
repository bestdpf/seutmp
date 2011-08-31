/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: GyroRate.cpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#include "GyroRate.h"
#include "robot/humanoid/Humanoid.h"
 
namespace perception {

GyroRate::GyroRate()
{
}

GyroRate::~GyroRate()
{
}

/* (GYR (name torsogyro) (-0.12 -0.01 -0.89))  */
bool GyroRate::update(const sexp_t* sexp)
{
	// get the name
	std::string name;
	if ( !parser::SexpParser::parseGivenValue(sexp,SS_NAME_STR,name) )
	{
		std::cerr<<"can not get the GYR name!\n";
		return false;
	}
	
	// get the id
    int id = HUMANOID.getGyroRateId(name);
	if ( id < 0 )
	{
		std::cerr<<"unknown GYR name: "<<name<<'\n';
		return false;
	}
	
	// get the rt
	sexp = sexp->next;
    math::Vector3f rate;
    if ( !parser::SexpParser::parseGivenValue(sexp,"rt",rate) )
	{
		std::cerr<<"can not get GYR rt value\n";
		return false;
	}
	
	// set the rt
	mRates[id] = rate;
	return true;
}

std::ostream& operator<<(std::ostream &stream, const GyroRate& g)
{
	stream<<"(GYR ";
	for( GyroRate::TRateMap::const_iterator iter = g.mRates.begin(); iter!=g.mRates.end(); ++iter )
	{
		stream<<"(name "<<HUMANOID.getGyroRateName(iter->first)<<")("<<iter->second<<")";
	}
	stream<<")";
	return stream;
}

} // namespace perception
