/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef SERVER_SETTING_SOCCER_DEFINES_H
#define SERVER_SETTING_SOCCER_DEFINES_H

/** this file contains types common to all plugins in the soccer simulation */

#include <string>
#include "math/Math.hpp"

namespace serversetting {

using namespace math;

enum TPlayMode
{
    // the order of the first 3 play modes should not be changed.
    PM_BEFORE_KICK_OFF = 0,      /*!< before_kick_off:   before the match  */
    PM_KICK_OFF_LEFT = 1,       /*!< kick_off_left:     kick off for the left team  */
    PM_KICK_OFF_RIGHT = 2,      /*!< kick_off_right:    kick off for the right team */
    PM_PLAY_ON,                 /*!< play_on:           regular game play */
    PM_KICK_IN_LEFT,
    PM_KICK_IN_RIGHT,
    PM_CORNER_KICK_LEFT,       /*!< corner_kick_l:     corner kick left team   */
    PM_CORNER_KICK_RIGHT,      /*!< corner_kick_r:     corner kick right team  */
    PM_GOAL_KICK_LEFT,         /*!< goal_kick_l:       goal kick for left team */
    PM_GOAL_KICK_RIGHT,        /*!< goal_kick_r:       goal kick for right team*/
    PM_OFFSIDE_LEFT,           /*!< offside_l:         offside for left team   */
    PM_OFFSIDE_RIGHT,          /*!< offside_r:         offside for right team  */
    PM_GAME_OVER,
    PM_GOAL_LEFT,
    PM_GOAL_RIGHT,
    PM_FREE_KICK_LEFT,         /*!< free_kick_l:       free kick for left team */
    PM_FREE_KICK_RIGHT,        /*!< free_kick_r:       free kick for right team*/
    PM_NULL                    /*!< no play mode, this must be the last entry */
};

/*!The ObjectSetT enumerations holds the different object sets, which
   consists of one or multiple ObjectT types. */
enum ObjectSetT
{
  OBJECT_SET_TEAMMATES,        /*!< teammates                       */
  OBJECT_SET_TEAMMATES_NO_ME,  /*!< teammates   no  me              */
  OBJECT_SET_OPPONENTS,        /*!< opponents                       */
  OBJECT_SET_PLAYERS,          /*!< players  include teammates and opponents */
  OBJECT_SET_PLAYERS_NO_ME,    /*!< players  include teammates and opponents  except me*/

  OBJECT_SET_TEAMMATES_NO_GOALIE,/*!< teammates without the goalie  */
  OBJECT_SET_FLAGS,            /*!< flags                           */
  OBJECT_SET_LINES,            /*!< lines                           */
  OBJECT_SET_ILLEGAL           /*!< illegal                         */
} ;







/** mapping from TPlayMode to string constants */
#define STR_PM_BeforeKickOff "BeforeKickOff"
#define STR_PM_KickOff_Left "KickOff_Left"
#define STR_PM_KickOff_Right "KickOff_Right"
#define STR_PM_PlayOn "PlayOn"
#define STR_PM_KickIn_Left "KickIn_Left"
#define STR_PM_KickIn_Right "KickIn_Right"
#define STR_PM_CORNER_KICK_LEFT "corner_kick_left"
#define STR_PM_CORNER_KICK_RIGHT "corner_kick_right"
#define STR_PM_GOAL_KICK_LEFT "goal_kick_left"
#define STR_PM_GOAL_KICK_RIGHT "goal_kick_right"
#define STR_PM_OFFSIDE_LEFT "offside_left"
#define STR_PM_OFFSIDE_RIGHT "offside_right"
#define STR_PM_GAME_OVER "GameOver"
#define STR_PM_GOAL_LEFT "Goal_Left"
#define STR_PM_GOAL_RIGHT "Goal_Right"
#define STR_PM_FREE_KICK_LEFT "free_kick_left"
#define STR_PM_FREE_KICK_RIGHT "free_kick_right"
#define STR_PM_NULL "unknown"

    template<typename T>
    TPlayMode getPlayModeByName(const T& name)
    {
        if ( STR_PM_PlayOn == name )
            return PM_PLAY_ON;
        if ( STR_PM_BeforeKickOff == name )
            return PM_BEFORE_KICK_OFF;
        if ( STR_PM_KickOff_Left == name )
            return PM_KICK_OFF_LEFT;
        if ( STR_PM_KickOff_Right == name )
            return PM_KICK_OFF_RIGHT;
        if ( STR_PM_KickIn_Left == name )
            return PM_KICK_IN_LEFT;
        if ( STR_PM_KickIn_Right == name )
            return PM_KICK_IN_RIGHT;
        if ( STR_PM_CORNER_KICK_LEFT == name )
            return PM_CORNER_KICK_LEFT;
        if ( STR_PM_CORNER_KICK_RIGHT == name )
            return PM_CORNER_KICK_RIGHT;
        if ( STR_PM_GOAL_KICK_LEFT == name )
            return PM_GOAL_KICK_LEFT;
        if ( STR_PM_GOAL_KICK_RIGHT == name )
            return PM_GOAL_KICK_RIGHT;
        if ( STR_PM_OFFSIDE_LEFT == name )
            return PM_OFFSIDE_LEFT;
        if ( STR_PM_OFFSIDE_RIGHT == name )
            return PM_OFFSIDE_RIGHT;
        if ( STR_PM_GAME_OVER == name )
            return PM_GAME_OVER;
        if ( STR_PM_GOAL_LEFT == name )
            return PM_GOAL_LEFT;
        if ( STR_PM_GOAL_RIGHT == name )
            return PM_GOAL_RIGHT;
        if ( STR_PM_FREE_KICK_LEFT == name )
            return PM_FREE_KICK_LEFT;
        if ( STR_PM_FREE_KICK_RIGHT == name )
            return PM_FREE_KICK_RIGHT;

        return PM_NULL;
    }

    template<typename T>
    std::string getPlayModeName(T pm)
    {
        switch(pm){
        case PM_BEFORE_KICK_OFF:
            return STR_PM_BeforeKickOff;
        case PM_KICK_OFF_LEFT:
            return STR_PM_KickOff_Left;
        case PM_KICK_OFF_RIGHT:
            return STR_PM_KickOff_Right;
        case PM_PLAY_ON:
            return STR_PM_PlayOn;
        case PM_KICK_IN_LEFT:
            return STR_PM_KickIn_Left;
        case PM_KICK_IN_RIGHT:
            return STR_PM_KickIn_Right;
        case PM_CORNER_KICK_LEFT:
            return STR_PM_CORNER_KICK_LEFT;
        case PM_CORNER_KICK_RIGHT:
            return STR_PM_CORNER_KICK_RIGHT;
        case PM_GOAL_KICK_LEFT:
            return STR_PM_GOAL_KICK_LEFT;
        case PM_GOAL_KICK_RIGHT:
            return STR_PM_GOAL_KICK_RIGHT;
        case PM_OFFSIDE_LEFT:
            return STR_PM_OFFSIDE_LEFT;
        case PM_OFFSIDE_RIGHT:
            return STR_PM_OFFSIDE_RIGHT;
        case PM_GAME_OVER:
            return STR_PM_GAME_OVER;
        case PM_GOAL_LEFT:
            return STR_PM_GOAL_LEFT;
        case PM_GOAL_RIGHT:
            return STR_PM_GOAL_RIGHT;
        case PM_FREE_KICK_LEFT:
            return STR_PM_FREE_KICK_LEFT;
        case PM_FREE_KICK_RIGHT:
            return STR_PM_FREE_KICK_RIGHT;
        default: return STR_PM_NULL;
        }
    }

enum TTeamIndex
{
    TI_LEFT = 0,
    TI_RIGHT,
	TI_NULL /*< this must be the last entry >*/
};

#define STR_TI_LEFT "left"
#define STR_TI_RIGHT "right"
#define STR_TI_NULL "unknown"

    template<typename T>
    TTeamIndex getTeamIndexByName(const T& name)
    {
        if (name == STR_TI_LEFT)
            return TI_LEFT;
        if (name == STR_TI_RIGHT)
            return TI_RIGHT;
        return TI_NULL;
    }

    template<typename T>
    std::string getTeamIndexName(T ti)
    {
        switch(ti){
        case TI_LEFT:
            return STR_TI_LEFT;
        case TI_RIGHT:
            return STR_TI_RIGHT;
        default:
            return STR_TI_NULL;
        }
    }

enum TGameHalf
{
    GH_FIRST = 0,
    GH_SECOND,
	GH_NULL /*< this must be the last entry >*/
};

/////////////// Const Parameters /////////////

/** field size */
const float field_length = 21.0f;
    const float half_field_length = field_length*0.5f;
const float field_width = 14.0f;
    const float half_field_width = field_width*0.5f;
const float field_height = 40.0f;
const float border_size = 0.0f; //TT: from naosoccersim.rb: #prevent complaining about missing variable
    const float free_kick_distance = 1.3f;

    /** penalty size */
    const float penalty_length = 2.1f;
    const float penalty_width = 6.2f;
    const float half_penalty_width = penalty_width * 0.5f;

/** goal size */
const float goal_width = 2.1f;
    const float half_goal_width = goal_width*0.5f;
const float goal_height = 0.8f;
const float goal_depth = 0.6f;

/** flag */
const float flag_height = 0.0;
const float flag_border_size = 2;

/** the ball */
const float ball_radius = 0.042f;
const float ball_mass = 0.026f;
const float ball_angular_drag = 0.00005f;
const float ball_linear_drag = 0.01f;

/** simulation parameters */
const float sim_step = 0.02;
const float acceleration_of_gravity = 9.81f;

/** time of rule */
const float rule_goal_pause_time = 3.0f;
const float rule_kick_in_pause_time = 1.0;
const float rule_half_time = 5.0 * 60;
const float rule_drop_ball_time = 30;
const float wait_before_kick_off = 2.0;


/** players */
const unsigned int teammate_nums = 11;
const unsigned int opponent_nums = 11;

} // namespace serversetting

#endif // SERVER_SETTING_SOCCER_DEFINES_H
