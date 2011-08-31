/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
 
#include "ForceResistance.h"
#include "robot/humanoid/Humanoid.h"

namespace perception {

    using namespace std;
    
ForceResistance::ForceResistance()
{
}

ForceResistance::~ForceResistance()
{
}

/* (FRP (n rf) (c -0.10 0.09 -0.06) (f 12.47 -8.75 0.00)) */
bool ForceResistance::update(const sexp_t* sexp)
{   
	// get the name
	string name;
	if ( !parser::SexpParser::parseGivenValue(sexp,SS_NAME_STR,name) )
	{
		std::cerr<<"can not get the FRP name!"<<endl;
		return false;
	}
	
	// get the id
	int id = HUMANOID.getForceResistanceId(name);
	if ( id < 0 )
	{
		std::cerr<<"unknown FRP name: "<<name<<endl;
		return false;
	}

	FeedBack fd;
	// get the center position
	sexp = sexp->next;
	if ( !parser::SexpParser::parseGivenValue(sexp,"c",fd.pos) )
	{
		std::cerr<<"can not get the FRP c! "<<endl;
		return false;
	}
	
	// get the force
	sexp = sexp->next;
	if ( !parser::SexpParser::parseGivenValue(sexp,"f",fd.force) )
	{
		std::cerr<<"can not get the FRP c! "<<endl;
		return false;
	}
	
	// store the val
	mVals[id] = fd;
	return true;
}

std::ostream& operator<<(std::ostream &stream, const ForceResistance& fr)
{
	stream<<"(FRP ";
	for( ForceResistance::TValMap::const_iterator iter = fr.mVals.begin(); iter!=fr.mVals.end(); ++iter )
	{
		stream<<"(n "<<HUMANOID.getForceResistanceName(iter->first)<<" (c "<<iter->second.pos<<")(f "<<iter->second.force<<"))";
	}
	stream<<")";
	return stream;
}

} // namespace perception
