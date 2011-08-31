/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
 
#include "GameState.h"
#include "Vision.h"
#include "FieldInfo.h"
namespace perception {

//////////// static values ////////////
unsigned int GameState::mUnum = 0;
serversetting::TTeamIndex GameState::mTeamIndex = serversetting::TI_NULL;
	
GameState::GameState()
:mGameTime(0)
{
}

GameState::~GameState()
{
}

// "(GS (t 0.00) (pm BeforeKickOff))"
bool GameState::update(const sexp_t* sexp)
{
	bool ok = true;
	std::string name;
	while ( sexp )
	{
		const sexp_t* t = sexp->list;
		if ( parser::SexpParser::parseValue(t,name) )
		{
			if ( "t" == name ) // time
			{
				if ( !parser::SexpParser::parseValue(t->next,mGameTime) )
				{
					ok = false;
					std::cerr<<"GameState::update failed get time value\n";
				}
			}
			else if ( "pm" == name ) // play mode
			{
				std::string pm;
				if ( !parser::SexpParser::parseValue(t->next,pm) )
				{
					ok = false;
					std::cerr<<"GameState::update failed get play mode value\n";
				}
				mPlayMode = serversetting::getPlayModeByName(pm);
			}
			else if ( "unum" == name ) // unum
			{
				if ( !parser::SexpParser::parseValue(t->next,mUnum) )
				{
					ok = false;
					std::cerr<<"GameState::update failed get unum value\n";
				}
			}
			else if ( "team" == name ) // side
			{
				std::string team;
				if ( !parser::SexpParser::parseValue(t->next,team) )
				{
					ok = false;
					std::cerr<<"GameState::update failed get team index value\n";
				}
				setOurTeamIndex( serversetting::getTeamIndexByName(team) );
			}
			else
			{
				ok = false;
				std::cerr<<"GameState::update unknown name: "<<name<<'\n';
			}
		}
		else
		{
			ok = false;
			std::cerr<<"GameState::update can not get the name!\n";
		}
		sexp = sexp->next;
	}
	
	return ok;
}

void GameState::setOurTeamIndex(serversetting::TTeamIndex ti)
{
    mTeamIndex = ti;
    Vision::setupObjectsVision(ti);
    FIELD2.setupFieldInfo(ti);
}

std::ostream& operator<<(std::ostream &stream, const GameState& g)
{
	stream << "(GS "
           << "(t " << g.mGameTime << ')'
           << "(pm " << serversetting::getPlayModeName(g.mPlayMode) << ')'
           << "(team " << serversetting::getTeamIndexName(g.mTeamIndex) << ')'
           << "(unum " << g.mUnum <<')';
	return stream;
}

} // namespace perception
