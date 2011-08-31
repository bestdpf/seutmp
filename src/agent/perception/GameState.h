/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
 

#ifndef PERCEPTION_GAME_STATE_H
#define PERCEPTION_GAME_STATE_H

#include "math/Math.hpp"
#include "BasicPerception.h"
#include "SoccerDefines.h"


namespace perception {
	
class GameState: public BasicPerception
{
public:
	GameState();
	~GameState();
	
	virtual bool update(const sexp_t* sexp);

	friend std::ostream& operator<<(std::ostream &stream, const GameState& g);

	static unsigned int unum() { return mUnum; }

    serversetting::TPlayMode getPlayMode() const
        {
            return mPlayMode;
        }

    void setPlayMode(serversetting::TPlayMode pm)
        {
            mPlayMode = pm;
        }

    static serversetting::TTeamIndex getOurTeamIndex()
        {
            return mTeamIndex;
        }

    static void setOurTeamIndex(serversetting::TTeamIndex ti);

    float gameTime() const
        {
            return mGameTime;
        }

private:
	/** game time */
	float mGameTime;

	/** play mode */
	serversetting::TPlayMode mPlayMode;
	
	////////////// static values ////////////
	/** my unum */
	static unsigned int mUnum;
	static serversetting::TTeamIndex mTeamIndex;
};

std::ostream& operator<<(std::ostream &stream, const GameState& g);

} // namespace perception

#endif // PERCEPTION_GAME_STATE_H
