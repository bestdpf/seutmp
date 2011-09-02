/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef SOCCER_PLAYER_H
#define SOCCER_PLAYER_H

#define ENABLE_PLAYER_LOG

#include "configuration/Configuration.h"
#include "task/Task.h"
#include "core/Agent.h"
#include "task/KeepBalance.h"
#include "task/Kick.h"
#include "task/Fall.h"
#include "SoccerDefines.h"
#include "task/BasicKick.h"
#include "perception/Message.h"

#ifdef ENABLE_PLAYER_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace soccer {

    using namespace math;
    using namespace boost;

    enum GoalKeeperState {
        LIED_STATE = 0, //lied state
        LYING_STATE, //lying state
        DIVED_STATE, //dived state
        DIVING_STATE, //diving state
        //LR_ROLLED_STATE,				//rocking state
        LEFTFALL_STATE,
        RIGHTFALL_STATE,
        BALANCE_STATE //balance state
    };

    struct KickMotion {
        std::string firstTaskName;
        math::Vector2f kickTargetRel;
        math::Vector2f myDesiredRelPosToBall; //used in calculation of myDesiredPos, related to WM.getBallGlobalPos2D()
        math::Vector2f relPosToBallToStopWalk; //used in judgement of stopping walk, related to WM.getBallRelPos2D()
    };

    class Player : public core::Agent {
    public:
        Player();

        virtual ~Player();

        /** initalization the player */
        virtual bool init();

        virtual boost::shared_ptr<action::Action> mysay(int p_recvNum, bool p_isFall, float p_bx, float p_by, float p_rx, float p_ry); //allen add it
        /** think what need to do, i.e make the decision */
        virtual boost::shared_ptr<action::Action> think();
    protected:
        ///////// interface for TeamPlayer ////////////
        /** the paly-on mode, mainly loop */
        virtual boost::shared_ptr<action::Action> playPlayOn() = 0;

        /** before kick off */
        virtual boost::shared_ptr<action::Action> playBeforeKickOff() = 0;

        /** kick off */
        boost::shared_ptr<action::Action> playKickOff();

        virtual boost::shared_ptr<action::Action> playOurKickOff() = 0;

        virtual boost::shared_ptr<action::Action> playOppKickOff() = 0;

        /** kick in */
        boost::shared_ptr<action::Action> playKickIn();

        virtual boost::shared_ptr<action::Action> playOurKickIn() = 0;

        virtual boost::shared_ptr<action::Action> playOppKickIn() = 0;

        /** corner kick */
        boost::shared_ptr<action::Action> playCornerKick();

        virtual boost::shared_ptr<action::Action> playOurCornerKick() = 0;

        virtual boost::shared_ptr<action::Action> playOppCornerKick() = 0;

        /** goal kick */
        boost::shared_ptr<action::Action> playGoalKick();

        virtual boost::shared_ptr<action::Action> playOurGoalKick() = 0;

        virtual boost::shared_ptr<action::Action> playOppGoalKick() = 0;

        /** offside */
        boost::shared_ptr<action::Action> playOffSide();

        virtual boost::shared_ptr<action::Action> playOurOffSide() = 0;

        virtual boost::shared_ptr<action::Action> playOppOffSide() = 0;

        /** game over */
        virtual boost::shared_ptr<action::Action> playGameOver() = 0;

        /** Gooooooooooooooooal */
        boost::shared_ptr<action::Action> playGoal();

        virtual boost::shared_ptr<action::Action> playOurGoal() = 0;

        virtual boost::shared_ptr<action::Action> playOppGoal() = 0;

        /** free kick */
        boost::shared_ptr<action::Action> playFreeKick();

        virtual boost::shared_ptr<action::Action> playOurFreeKick() = 0;

        virtual boost::shared_ptr<action::Action> playOppFreeKick() = 0;

    public:
        /************* Skills **************/
        /**
         * let the robot walk to a desired position with desired direction
         *
         * @param stopPos the desired stop position
         * @param dir the direction when reached the desired position
         *
         * @return current action
         */
        boost::shared_ptr<action::Action> keepGoal(); //Skill of GoalKeeper
        boost::shared_ptr<action::Action> keepGoal_TT(int d); //TT keepGoal


        task::Direction calFallDirection(); //Skill of GoalKeeper
        task::Direction calFallDirection(int n);
        task::Direction calFallDirPenalty();
        bool isBallWillCrossMeAfterNStps(int n);

        bool isGoal(); //

        GoalKeeperState updateGoalKeeperState(); //

        boost::shared_ptr<action::Action> goTo(const math::Vector2f& destPos, math::AngDeg bodyDir, bool avoidBall = true);

        boost::shared_ptr<action::Action> goToRel(const math::Vector2f& target, math::AngDeg dir);

        //bodyDir: body direction
        //one body direction may match any walk direction
        boost::shared_ptr<action::Action> goToAvoidBlocks(math::Vector2f dest, math::AngDeg bodyDir, bool avoidBall = true);

        boost::shared_ptr<action::Action> goToSlow(const math::Vector2f& stopPos,
                math::AngDeg dir, bool avoidBall = true, bool is4Kick = false);
        /**
         * walk to the desired position and look at a given position
         *
         * @param stopPos the desired stop position
         * @param lookAt the global position of looking at
         *
         * @return current action
         */
        boost::shared_ptr<action::Action> goTo(const math::Vector2f& stopPos,
                const math::Vector2f& lookAt, bool avoidBall = true);

        boost::shared_ptr<action::Action> goToSlow(const math::Vector2f& stopPos,
                const math::Vector2f& lookAt, bool avoidBall = true, bool is4Kick = false);

        boost::shared_ptr<action::Action> kickTo(const math::Vector2f& goal, bool useMaxForceMotion = true);

        //TT, MMXI
        boost::shared_ptr<action::Action> kickRel();
        boost::shared_ptr<action::Action> shootRel();
        boost::shared_ptr<action::Action> testActionTT();
        boost::shared_ptr<action::Action> dribbleRel();
        boost::shared_ptr<action::Action> penaltyKiller(); //Gravity penalty killer;
        boost::shared_ptr<action::Action> dribbleToOppGoal();
        boost::shared_ptr<action::Action> goToBallBack();
        boost::shared_ptr<action::Action> kickToRel(const math::Vector2f targetRel);
        boost::shared_ptr<action::Action> passToPlayer(unsigned int num);
        unsigned int choosePassPlayer();
        boost::shared_ptr<action::Action> interceptionBall();
        boost::shared_ptr<action::Action> sideWalk(bool isLeft);
        math::Vector2f GlobalToRel(const math::Vector2f& global);
        boost::shared_ptr<action::Action> kickToBetweenRel(const math::Vector2f& leftBoundary, const math::Vector2f& rightBoundary);
        boost::shared_ptr<action::Action> breakingBall();

        boost::shared_ptr<action::Action> kickBetween(const math::Vector2f& goalLeft,
                const math::Vector2f& goalRight);


        //TT
        // +-1, +-2
        boost::shared_ptr<action::Action> fallToGetBall(int dir);


        boost::shared_ptr<action::Action>
        beamAndInit(const math::Vector3f& beamPos);


        boost::shared_ptr<action::Action> dribble();

        //TT
        //dribble to [angC] direction, between [angL] and [angR]
        //all of them are global parameters
        boost::shared_ptr<action::Action> dribbleToDir(AngDeg angC, AngDeg angL, AngDeg angR);

        /**
         * the player have to kick the ball away as soon as possible, the
         * player just rush to the ball in this state
         *
         * @return the action
         */
        boost::shared_ptr<action::Action> clearBall();

        /**
         * return an action that passing the ball to teammate
         *
         * @return the passing action
         */
        boost::shared_ptr<action::Action> pass(const bool waklSlow = false);

        /**
         * the function for challenge of walking
         */
        boost::shared_ptr<action::Action> walkToBall();

        /*     float calKickHeight(const math::Vector2f& target,
                     const math::Vector2f& ballPos,
                     task::Kick::KickMode mode);*/

        math::Vector2f chooseLargestAttackAngle(float x, float miny, float maxy,
                const math::Vector2f& pos, math::AngDeg& maxAng);

        /**
         * just shoot
         */
        boost::shared_ptr<action::Action> shoot();

        /**
         * the behavior of defender: do not try to kick the ball directly
         * while opponent is faster to ball than me, just run to the
         * defense position to keep a good situation
         *
         * @return
         */
        boost::shared_ptr<action::Action> defense();

        boost::shared_ptr<action::Action> runDefensePos();

        /**
         * choose the kick foot according to current state
         *
         * @param goal where want to kick to
         *
         * @return if chose the left foot kick, otherwise use right foot
         */
        bool chooseKickFoot(const math::Vector2f& goal);

        bool chooseKickFoot2(const math::Vector2f& goal);

        /**
         * choose the kick type according to current state
         *
         * @param goal where want to kick to
         *
         * @return a shared_ptr<BasicKick>
         */
        boost::shared_ptr<task::BasicKick> chooseKickType(const math::Vector2f& goal, bool useMaxForceMotion, Vector2f* pPos, AngDeg* pDir, Vector2f* pRelPosToStopWalk);

        /*!	is there any opponent in the front of me?
          ie. Is there any opponent in the rectangle which in my heading direction?
          \return the number of block opponent, 0 means no opponent blocks me
         */
        int isSpaceAhead(const math::Vector2f& goal, float dist);

        /**
         * parse the hear message: whether a teammate is passing ball to
         * me
         */
        bool isPassingToMe();
        bool whetherToFall();
        /**
         * calculate the best pass point to a teammate
         *
         * @param num the number of teammate
         *
         * @return the best pass point
         */
        math::Vector2f calPassPoint(unsigned int num);

        //////////////////////////////////
        /// FOR TEST ONLY
        //////////////////////////////////
        /**
         * beam to a random position in the field, this is used to test
         * the agent's localization
         *
         * @return
         */
        boost::shared_ptr<action::Action> randomBeam();

        bool amIFreeToShoot(int num); //terrymimi

        bool isBallWillInAfterNSteps(int n);
        void myhear();
        configuration::Formation::FormationType chooseFormation();
    protected:

        /// cache the task
        task::Task mTask;

        task::KeepBalance mBalance;

        //TT, April, MMXI
        //-1: as CM wish
        //0: don't turn
        //1: search ball
        //2: stick to ball
        //3: search flags
        //4: for attacking (focus on ball and opp goal)
        //-2: for test
        int mCameraMotionMode;

        //true: kicking, don't want to change my mind
        bool mKickLock;

        //TT, April, MMXI
        //0: as LR wish
        //1: left
        //2: right
        //int mMovingDir;

    private:
        DECLARE_GRAPHIC_LOGGER;
        math::Vector3f mBallPredictedPos;
        std::vector<KickMotion> mKickMotionVector;

    }; //end of class Player


} //end of namespace soccer

#endif //SOCCER_PLAYER_H

