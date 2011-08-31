/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Touch.cpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
#include "Touch.h"
#include "robot/humanoid/Humanoid.h"
 
namespace perception {
	
Touch::Touch()
{
}

Touch::~Touch()
{
}

/* (TCH (n rf) (val 0)) */
bool Touch::update(const sexp_t* sexp)
{
	// get the name
	std::string name;
	if ( !parser::SexpParser::parseGivenValue(sexp,SS_NAME_STR,name) )
	{
		std::cerr<<"can not get the TCH name! \n";
		return false;
	}
	
	// get the id
	int id = HUMANOID.getTouchId(name);
	if ( id < 0 )
	{
		std::cerr<<"unknown TCH name: "<<name<<'\n';
		return false;
	}

	// get the val
	sexp = sexp->next;
	int val = 0;
	if ( !parser::SexpParser::parseGivenValue(sexp,"val",val) )
	{
		std::cerr<<"can not get the TCH val! \n";
		return false;
	}
	
	// set the val
	mVals[id] = val;
	return true;
}

std::ostream& operator<<(std::ostream &stream, const Touch& t)
{
	stream<<"(TCH ";
	for( Touch::TValMap::const_iterator iter = t.mVals.begin(); iter!=t.mVals.end(); ++iter )
	{
		stream<<"(name "<<HUMANOID.getTouchName(iter->first)<<")(val "<<iter->second<<")";
	}
	stream<<")";
	return stream;
}

} // namespace perception
