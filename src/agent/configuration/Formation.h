/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef SOCCER_FORMATION_H
#define SOCCER_FORMATION_H

#include "Template.hpp"
#include "math/Math.hpp"
#include "parser/SexpParser.hpp"
#include "ClassException.hpp"

namespace configuration {
 
class Formation
{
public: 
	Formation();

	~Formation();

	//# We have the following players types:
	//# -----------------------------------
	enum PlayerType
	{
		PT_GOALKEEPER = 0,
		PT_DEFENDER_CENTRAL,
		PT_DEFENDER_SWEEPER,
		PT_DEFENDER_WING,
		PT_MIDFIELDER_SWEEPER,
		PT_MIDFIELDER_CENTER,
		PT_MIDFIELDER_WING,
		PT_ATTACKER_WING,
		PT_ATTACKER_CENTRAL,
		PT_JOYSTICK,
        PT_NULL
    };
    //# We have the following Formation types:
	//# Allen-----------------------------------
    enum FormationType
    {
        FT_HOME=0,
        FT_ATTACK_MIDDLE,
        FT_ATTACK_RIGHT,
        FT_ATTACK_LEFT,
        FT_DEFEND_MIDDLE,
        FT_DEFEND_RIGHT,
        FT_DEFEND_LEFT,
        FT_NULL
    };
	/** data struct of one palyer's formation */
	struct FormationData
	{
        std::string robot;
        PlayerType type;
        math::Vector3f beforeKickOffBeam; // (x, y, o)
        math::Vector3f ourGoalBeam;
        math::Vector3f oppGoalBeam;
        math::Vector3f oppGoalKick;
        math::Vector3f homePos;
         math::Vector3f attackPos;
          math::Vector3f defendPos;
        math::Vector2f attraction;
        math::Vector2f xRange;
        math::Vector2f yRange;
		float	behindBall;
		//Vector2f _ourGoalKickPosLeft;// position when our goal kick to the left side
		//Vector2f _ourGoalKickPosRight;//position when our goal kick to the right side
		//Vector2f _ourGoalieCatchedPos;//position when our goalie catched the ball
	};

    void loadFile(const std::string& filename) throw(ClassException<Formation>);

    /** set the my formation data pointer by the unum and the formation name */
    void setMyFormation(const std::string& formationName, unsigned int num)
        throw(ClassException<Formation>);

    const FormationData& getMy() const 
        { return *mMyDataPtr;}

    
    static math::Vector3f calStrategicPosition( const FormationData& fdata,
                                                const math::Vector3f& posBall );
    

    math::Vector3f calMyStrategicPos( const math::Vector3f& posBall ) const;
   
 private:
    void setupStrPlayerTypeMap();
    
    void parseOneFormation(const sexp_t* sexp)throw(ClassException<Formation>);
    
    FormationData parseFormationData(const sexp_t* sexp) const throw(ClassException<Formation>);

    PlayerType getPlayerTypeByStr(const std::string& str) const
        { return getSecondValueByFirstRef(mStrPlayerTypeMap,str); }
    
	/** formation data */
    typedef std::map<std::string, std::map<unsigned int, FormationData> >
    TFormationDataMap;
    TFormationDataMap mDatas;

    /**  mapping from string name to PlayerType */
    typedef std::map<std::string, PlayerType> TStrPlayerTypeMap;
    TStrPlayerTypeMap mStrPlayerTypeMap;

    const FormationData* mMyDataPtr;
 

};

} // namespace configuration

#endif // SOCCER_FORMATION_H
