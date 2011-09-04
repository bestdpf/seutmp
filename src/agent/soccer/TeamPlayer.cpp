/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: TeamPlayer.cpp 2755 2009-04-02 08:00:08Z zyj $
 *
 ****************************************************************************/

#include "configuration/Configuration.h"
#include "core/WorldModel.h"
#include "action/BeamAction.h"
#include "action/Actions.h"
#include "controller/FixedAngleTrace.h"
#include "TeamPlayer.h"
#include "task/KeepBalance.h"
#include "task/Kick.h"
#include "perception/Vision.h"
#include "task/CameraMotion.h"
#include "core/PassModel.h"


namespace soccer {

    using namespace std;
    using namespace boost;
    using namespace serversetting;
    using namespace action;
    using namespace task;
    using namespace math;

    TeamPlayer::TeamPlayer() {
    }

    TeamPlayer::~TeamPlayer() {
    }

    bool TeamPlayer::init() {
        if (!Player::init()) return false;

        // get the game state information from the server
        // such as team index, unum, etc
        while (true) {
            boost::shared_ptr<perception::Perception> p = sense();
            if (0 == p.get()) break;
            if (!WM.update(p)) break;
            if (WM.getMyUnum() > 0) {
                shared_ptr<Action> act = beamAndInit(FM.getMy().beforeKickOffBeam);
                perform(act);

                return true;
            }
        }
        return false;
    }

    /** the paly-on mode, mainly loop */
    shared_ptr<Action> TeamPlayer::playPlayOn() { //return  goalKeeperBehaviour();
        /*************************************
         * EXPERIMENTS:
         * We do some experiments here
         *************************************/


        if (WM.isGameStateChanged())
            mTask.clear();


        shared_ptr<Action> act;

        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER: //No.1
                act = goalKeeperBehaviour();
                break;
            case configuration::Formation::PT_ATTACKER_CENTRAL: //No.9
                act = attackerCentralBehaviour();
                break;
            case configuration::Formation::PT_ATTACKER_WING: //No.8
                act = attackerLeftWingBehaviour();
                break;
            case configuration::Formation::PT_MIDFIELDER_WING: //No.7
                act = attackerRightWingBehaviour();
                break;
            case configuration::Formation::PT_MIDFIELDER_CENTER: //No.6
                act = middleFielderBehaviour();
                break;
            case configuration::Formation::PT_MIDFIELDER_SWEEPER://No.5
                act = defenderSweeperBehaviour();
                break;
            case configuration::Formation::PT_DEFENDER_CENTRAL: //No.2
                act = defenderCenteralBehaviour();
                break;
            case configuration::Formation::PT_DEFENDER_SWEEPER://No.4
                act = defenderLeftWingBehaviour();
                break;
            case configuration::Formation::PT_DEFENDER_WING: //No.3
                act = defenderRightWingBehaviour();
                break;
            default:
                act = defaultBehaviour();
                break;
        }
        return act;
    }

    /** before kick off */
    shared_ptr<Action> TeamPlayer::playBeforeKickOff() { ///terrymimi
        return beamAndInit(FM.getMy().beforeKickOffBeam + Vector3f(-0.2, 0.3, 0));
    }

    /** kick off */
    shared_ptr<Action> TeamPlayer::playOurKickOff() {
        shared_ptr<Action> act;
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f runtarget;
        switch (FM.getMy().type) {
            case configuration::Formation::PT_ATTACKER_CENTRAL:
            {
                runtarget.x() = 3;
                runtarget.y() = 3;
                act = kickTo(runtarget);
                break;
            }
            default:
                mBalance.perform();
                mBalance.clear();
                mTask.clear();
                // just keep position
                return FAT.controlPreferThan("squat", "*");
                break;
        }

        return act;
    }

    shared_ptr<Action> TeamPlayer::playOppKickOff() {
        mBalance.perform();
        mBalance.clear();
        mTask.clear();
        // just keep position
        return FAT.controlPreferThan("squat", "*");
    }

    /** kick in */
    shared_ptr<Action> TeamPlayer::playOurKickIn() {
        return playOurDeadBall();
    }

    shared_ptr<Action> TeamPlayer::playOppKickIn() {
        return playOppDeadBall();
    }

    /** corner kick */
    shared_ptr<Action> TeamPlayer::playOurCornerKick() {
        shared_ptr<Action> act;
        float delta1 = 0.3;
        float delta2 = 0.3;
        Vector2f BallPos = WM.getBallGlobalPos2D();
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER:
                act = goTo(calGoalKeeperDefensePos(), WM.getBallGlobalPos2D());
                break;
            default:
                act = playPlayOn();
                break;
        }

        return act;
    }

    shared_ptr<Action> TeamPlayer::playOppCornerKick() {
        return playOppDeadBall();
    }

    /** goal kick */
    shared_ptr<Action> TeamPlayer::playOurGoalKick() {
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f myPos = WM.getMyGlobalPos2D();
        Vector2f target(-half_field_length + 0.5, myPos.y());
        Vector2f oppgoal(0.0, 0.0);
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER:
            {
                const Vector2f& myPos = WM.getMyGlobalPos2D();
                const Vector2f& ballPos = WM.getBallGlobalPos2D();
                Vector2f Destination(-2, myPos.y() > 0 ? -2 : 2);
                Vector2f ourGoal(-half_field_length, 0);

                if (myPos.x() < ballPos.x()) {
                    return kickTo(Destination, true);
                } else {
                    return goTo(ourGoal, 0, true);
                }
                break;
            }
            default:
                return playPlayOn();
                break;
        }
    }

    shared_ptr<Action> TeamPlayer::playOppGoalKick() {
        return playOppDeadBall();
    }

    /** offside */
    shared_ptr<Action> TeamPlayer::playOurOffSide() {
        return playPlayOn();
    }

    shared_ptr<Action> TeamPlayer::playOppOffSide() {
        return playPlayOn();
    }

    /** game over */
    shared_ptr<Action> TeamPlayer::playGameOver() {
        cerr << "[ERROR] Player can not hanle playGameOver\n";
        return shared_ptr<Action > ();
    }

    /** Gooooooooooooooooal */
    shared_ptr<Action> TeamPlayer::playOurGoal() {
        return beamAndInit(FM.getMy().ourGoalBeam);
    }

    shared_ptr<Action> TeamPlayer::playOppGoal() {
        return beamAndInit(FM.getMy().oppGoalBeam);
    }

    /** free kick */
    shared_ptr<Action> TeamPlayer::playOurFreeKick() {
        return playOurDeadBall();
    }

    shared_ptr<Action> TeamPlayer::playOppFreeKick() {
        return playOurDeadBall();
    }

    shared_ptr<Action> TeamPlayer::runStrategicPos() {
        Vector3f strategicPos = FM.calMyStrategicPos(WM.getBallGlobalPos());
        Vector2f posStop(strategicPos.x(), strategicPos.y());
        math::AngDeg dir = WM.getMyBodyDirection();
        return goTo(posStop,dir);
    }


    shared_ptr<Action> TeamPlayer::defaultBehaviour() {
        /**
         * the simplest decision:
         * If I am the fastes, shoot the ball,
         * otherwise run to the strategic position
         */
        Vector2f oppGoal(half_field_length, 0);
        if (WM.amIFastestToBallOfOurTeam()) {
            unsigned bestPassId = choosePassPlayer();
            if (bestPassId != 0) {
                return passToPlayer(bestPassId);
            } else {
                return kickTo(oppGoal);
            }
        } else {
            return runStrategicPos();
        }
    }

    Vector2f TeamPlayer::calGoalKeeperDefensePos() {
        Vector2f goalLeft(-half_field_length, half_goal_width);
        Vector2f goalRight(-half_field_length, -half_goal_width);
        const Vector2f& posBall = WM.getBallGlobalPos2D();

        float distL = (goalLeft - posBall).length();
        float distR = (goalRight - posBall).length();
        float dist = distL > distR ? distR : distL;
        AngDeg angL = (goalLeft - posBall).angle();
        AngDeg angR = (goalRight - posBall).angle();
        AngDeg ang = calClipAng(angL, angR)*0.5f;
        AngDeg dir = calBisectorTwoAngles(angL, angR);
        dist *= cosDeg(ang);
        dist -= 0.5f;
        Vector2f p = posBall + pol2xyz(Vector2f(dist, dir));
        p.y() = clamp(p.y(), -half_goal_width + 0.055f, half_goal_width - 0.055f);
        return p;
    }

    shared_ptr<Action> TeamPlayer::goalKeeperBehaviour() //No.1
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::attackerCentralBehaviour() //No.9
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::attackerLeftWingBehaviour()//No.8
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::attackerRightWingBehaviour() //No.7
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::defenderSweeperBehaviour()//No.5
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::middleFielderBehaviour()//No.6
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::defenderRightWingBehaviour() //No.4
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::defenderCenteralBehaviour() //No.2
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::defenderLeftWingBehaviour() //No.3
    {
        return defaultBehaviour();
    }

    shared_ptr<Action> TeamPlayer::playOurDeadBall() {
        shared_ptr<Action> act;
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER:
                act = goTo(calGoalKeeperDefensePos(), WM.getBallGlobalPos2D());
                break;

            default:
                act = playPlayOn();
                break;
        }

        return act;
    }

    shared_ptr<Action> TeamPlayer::playOppDeadBall() {
        shared_ptr<Action> act;
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER:
            {
                Vector2f goalLeft(-half_field_length, half_goal_width);
                Vector2f goalRight(-half_field_length, -half_goal_width);
                const Vector2f& posBall = WM.getBallGlobalPos2D();
                float distL = (goalLeft - posBall).length();
                float distR = (goalRight - posBall).length();
                float dist = distL > distR ? distR : distL;
                AngDeg angL = (goalLeft - posBall).angle();
                AngDeg angR = (goalRight - posBall).angle();
                AngDeg ang = calClipAng(angL, angR)*0.5f;
                AngDeg dir = calBisectorTwoAngles(angL, angR);
                dist *= cosDeg(ang);
                dist -= 0.5f;
                Vector2f p = posBall + pol2xyz(Vector2f(dist, dir));
                p.y() = clamp(p.y(), -half_goal_width + 0.055f, half_goal_width - 0.055f);
                act = goTo(p, posBall);
                break;
            }
            default:
            {
                if (WM.amIFastestToBallOfOurTeam()) {
                    const Vector2f& posBall = WM.getBallGlobalPos2D();
                    const Vector2f goal(-half_field_length, 0);
                    AngDeg ang = (goal - posBall).angle();
                    Vector2f p = pol2xyz(Vector2f(free_kick_distance + 0.5f, ang)) + posBall;
                    return goTo(p, posBall);
                } else {
                    act = playPlayOn();
                }
                break;
            }
        }
        return act;
    }

    bool TeamPlayer::isDefenseStable() {
        const Vector2f& posBall = WM.getBallGlobalPos2D();
        float minX = -half_field_length;
        float maxX = posBall.x();
        float minY = -abs(posBall.y());
        float maxY = -minY;
        int defenseNum = 0;
        const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();

        FOR_EACH(iter, ourPos) {
            const Vector3f& p = iter->second;
            if (p.x() > minX && p.x() < maxX && p.y() > minY && p.y() < maxY) {
                defenseNum++;
            }
        }
        return defenseNum > 1;
    }

    bool TeamPlayer::shouldAllAttack() {
        float full_game_time = rule_half_time * 2;
        if (full_game_time - WM.getGameTime() < rule_half_time * 0.5) {
            // remain time is only 1/4 game time
            if (WM.getOurGoal() < WM.getOppGoal()) {
                return true;
            }
            if (WM.getOurGoal() == 0 && WM.getBallAveragePos().x() > 2) {
                return true;
            }
        }
        return false;
    }

    Vector2f TeamPlayer::calDefensePos(Vector2f& leftBlock, Vector2f& rightBlock) {

        const Vector2f& posBall = WM.getBallGlobalPos2D();
        float left_y = (leftBlock.y() - posBall.y()) / (leftBlock.x() - posBall.x()) * (-half_field_length - posBall.x()) + posBall.y();
        float right_y = (rightBlock.y() - posBall.y()) / (rightBlock.x() - posBall.x()) * (-half_field_length - posBall.x()) + posBall.y();
        left_y = left_y > half_goal_width ? half_goal_width : left_y;
        right_y = right_y>-half_goal_width ? right_y : -half_goal_width;
        Vector2f goalLeft(-half_field_length, left_y);
        Vector2f goalRight(-half_field_length, right_y);
        float distL = (goalLeft - posBall).length();
        float distR = (goalRight - posBall).length();
        float dist = distL > distR ? distR : distL;
        AngDeg angL = (goalLeft - posBall).angle();
        AngDeg angR = (goalRight - posBall).angle();
        AngDeg ang = calClipAng(angL, angR)*0.5f;
        AngDeg dir = calBisectorTwoAngles(angL, angR);
        dist *= cosDeg(ang);
        dist -= 0.5f;
        Vector2f p = posBall + pol2xyz(Vector2f(dist, dir));
        p.y() = clamp(p.y(), -half_goal_width + 0.055f, half_goal_width - 0.055f);
        return p;
    }

} // namespace soccer
