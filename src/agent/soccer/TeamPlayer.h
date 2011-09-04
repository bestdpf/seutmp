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

#include "Player.h"

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

        boost::shared_ptr<action::Action> playOurDeadBall();

        boost::shared_ptr<action::Action> playOppDeadBall();

        Vector2f calGoalKeeperDefensePos();

         /**
         * just shoot
         */
        boost::shared_ptr<action::Action> shoot();
        /**
         * the player have to kick the ball away as soon as possible, the
         * player just rush to the ball in this state
         *
         * @return the action
         */
        boost::shared_ptr<action::Action> clearBall();


    private:
    };

#define AGENT soccer::TeamPlayer::GetSingleton()

} // namespace soccer

#endif // SOCCER_TEAM_PLAYER_H
