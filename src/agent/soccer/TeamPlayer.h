/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef SOCCER_TEAM_PLAYER_H
#define SOCCER_TEAM_PLAYER_H

#define ENABLE_TEAM_PLAYER_LOG

#include "Player.h"
#ifdef ENABLE_TEAM_PLAYER_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace soccer {
    using namespace std;
    using namespace boost;
    using namespace math;

    class TeamPlayer : public Player, public Singleton<TeamPlayer> {
    public:
        TeamPlayer();

        virtual ~TeamPlayer();

        /** initalization the player */
        virtual bool init();

    protected:
        /** the paly-on mode, mainly loop */
        virtual boost::shared_ptr<action::Action> playPlayOn();

        /** before kick off */
        virtual boost::shared_ptr<action::Action> playBeforeKickOff();

        /** kick off */
        virtual boost::shared_ptr<action::Action> playOurKickOff();

        virtual boost::shared_ptr<action::Action> playOppKickOff();

        /** kick in */
        virtual boost::shared_ptr<action::Action> playOurKickIn();

        virtual boost::shared_ptr<action::Action> playOppKickIn();

        /** corner kick */
        virtual boost::shared_ptr<action::Action> playOurCornerKick();

        virtual boost::shared_ptr<action::Action> playOppCornerKick();

        /** goal kick */
        virtual boost::shared_ptr<action::Action> playOurGoalKick();

        virtual boost::shared_ptr<action::Action> playOppGoalKick();

        /** offside */
        virtual boost::shared_ptr<action::Action> playOurOffSide();

        virtual boost::shared_ptr<action::Action> playOppOffSide();

        /** game over */
        virtual boost::shared_ptr<action::Action> playGameOver();

        /** Gooooooooooooooooal */
        virtual boost::shared_ptr<action::Action> playOurGoal();

        virtual boost::shared_ptr<action::Action> playOppGoal();

        /** free kick */
        virtual boost::shared_ptr<action::Action> playOurFreeKick();

        virtual boost::shared_ptr<action::Action> playOppFreeKick();

    protected:
        boost::shared_ptr<action::Action> runStrategicPos();

        boost::shared_ptr<action::Action> runOpenPos();

        boost::shared_ptr<action::Action> defaultBehaviour();

        boost::shared_ptr<action::Action> attackerCentralBehaviour();

        boost::shared_ptr<action::Action> attackerRightWingBehaviour();

        boost::shared_ptr<action::Action> attackerLeftWingBehaviour();

        boost::shared_ptr<action::Action> middleFielderBehaviour();

        boost::shared_ptr<action::Action> defenderSweeperBehaviour();

        boost::shared_ptr<action::Action> defenderRightWingBehaviour();

        boost::shared_ptr<action::Action> defenderLeftWingBehaviour();

        boost::shared_ptr<action::Action> defenderCenteralBehaviour();

        boost::shared_ptr<action::Action> goalKeeperBehaviour();

        boost::shared_ptr<action::Action> changeRole(int onum);

        boost::shared_ptr<action::Action> playOurDeadBall();

        boost::shared_ptr<action::Action> playOppDeadBall();

        boost::shared_ptr<action::Action> defenseShoot();

        boost::shared_ptr<action::Action> playoffenderpenalty();

        boost::shared_ptr<action::Action> playdefenderpenalty();
        
        boost::shared_ptr<action::Action>playGK();

        bool isDefenseStable();

        bool shouldAllAttack();

        Vector2f calGoalKeeperDefensePos();

        Vector2f calDefensePos(Vector2f& leftBlock,Vector2f& rightBlock);
        
        bool isMeFastestToBall(); //both hear and see

        int getFastestPlayer(); //both hear and see


        ////////////////////////////////////
        unsigned int numNearBall();
        bool isBallInMyZone(Vector2f ballPos);
        bool isBallInMyZoneT(Vector2f ballPos);
        bool isBallInMyZoneTT(Vector2f ballPos);
        ////////////////////////////////////////


        ////////////////////////////////////////
        shared_ptr<action::Action> attack();

        shared_ptr<action::Action> runStrategicPosnew(Vector2f & target);

        bool suitableForShoot();
        bool suitableForPass();
        bool suitableForDribble();
        //unsigned int numNearBall();
        //bool isBallInMyZone(Vector2f ballPos);

        //////////////////////////////////////////

    private:
        /** whether i am passing in a single dead ball situation */
        bool mIsPassing;

        bool mAllAttack;

        DECLARE_GRAPHIC_LOGGER;
    };

#define AGENT soccer::TeamPlayer::GetSingleton()

} // namespace soccer

#endif // SOCCER_TEAM_PLAYER_H
