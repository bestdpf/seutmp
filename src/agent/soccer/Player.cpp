/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Player.cpp 2755 2009-04-02 08:00:08Z zyj $
 *
 ****************************************************************************/

#include "Player.h"
#include "core/WorldModel.h"
#include "action/InitAction.h"
#include "action/BeamAction.h"
#include "action/Say.h"
#include "action/Actions.h"
#include "configuration/Configuration.h"
#include "controller/FixedAngleTrace.h"
#include "task/WalkRel.h"
#include "task/Kick.h"
#include "math/TLine2.hpp"
#include "math/Math.hpp"
#include "math/TConvexPolygon.hpp"
#include "task/KickTask.h"
#include "task/CameraMotion.h"
#include "core/SayAndHearModel.h"
#include<fstream>
#include<time.h>

namespace soccer {

    using namespace std;
    using namespace boost;
    using namespace serversetting;
    using namespace controller;
    using namespace perception;
    using namespace action;
    using namespace task;

    Player::Player()
    : mTask(-1, NULL) {
        //load data of KickMotion
        mKickMotionVector.clear();
        ifstream inFile("data/kick_motion.txt", ios::in);
        if (NULL != inFile) {
            string oneLine;
            KickMotion tempKM;
            while (!inFile.eof()) {
                getline(inFile, oneLine); //maybe tempKM.firstTaskName

                if (oneLine.empty()) continue; //skip empty line
                if ('#' == oneLine[0] || ' ' == oneLine[0]) continue; //skip comment line

                tempKM.firstTaskName = oneLine;
                inFile >> tempKM.kickTargetRel.x() >> tempKM.kickTargetRel.y()
                        >> tempKM.myDesiredRelPosToBall.x() >> tempKM.myDesiredRelPosToBall.y()
                        >> tempKM.relPosToBallToStopWalk.x() >> tempKM.relPosToBallToStopWalk.y();

                mKickMotionVector.push_back(tempKM);
            }
            inFile.close();
        }
    }

    Player::~Player() {
    }

    bool Player::init() {
        if (!Agent::init()) return false;
        // get the respond (first message) from the server
        shared_ptr<Perception> p = sense();
        if (0 == p.get()) {
            return false;
        }
        if (!WM.update(p)) return false;
        // scend the init message
        shared_ptr<Action> iAct(new InitAction(OPTS.arg<string > ("teamname"),
                OPTS.arg<unsigned int>("unum")));
        perform(iAct);
        //Allen, for GUI under new server
        sense();
        shared_ptr<Action> bAct = shared_ptr<Action > (new BeamAction(FM.getMy().beforeKickOffBeam));
        perform(bAct);
        return true;
    }

    /**
     * the entry of "Think" thread
     * 1-GK shout
     * 2-different play mode
     * 3-CameraMotion
     *
     * @author Xu Yuan
     *
     * @return boost::shared_ptr<Action>
     */
    boost::shared_ptr<Action> Player::think() {
        boost::shared_ptr<Actions> actions(new Actions());
        if (SHM.IsCanSay()) {
            shared_ptr<Say> SHMsay(new Say(SHM.getSayString()));
            actions->add(SHMsay);
        }
        //TT add for controling CameraMotion
        mCameraMotionMode = -1; //mCameraMotionMode will be changed in "play mode"
        //play mode
        switch (WM.getPlayMode()) {
            case PM_BEFORE_KICK_OFF:
                actions->add(playBeforeKickOff());
                break;
            case PM_KICK_OFF_LEFT:
            case PM_KICK_OFF_RIGHT:
                actions->add(playKickOff());
                break;
            case PM_PLAY_ON:
                actions->add(playPlayOn());
                break;
            case PM_KICK_IN_LEFT:
            case PM_KICK_IN_RIGHT:
                actions->add(playKickIn());
                break;
            case PM_CORNER_KICK_LEFT:
            case PM_CORNER_KICK_RIGHT:
                actions->add(playCornerKick());
                break;
            case PM_GOAL_KICK_LEFT:
            case PM_GOAL_KICK_RIGHT:
                actions->add(playGoalKick());
                break;
            case PM_OFFSIDE_LEFT:
            case PM_OFFSIDE_RIGHT:
                actions->add(playOffSide());
                break;
            case PM_GAME_OVER:
                actions->add(playGameOver());
                break;
            case PM_GOAL_LEFT:
            case PM_GOAL_RIGHT:
                actions->add(playGoal());
                break;
            case PM_FREE_KICK_LEFT:
            case PM_FREE_KICK_RIGHT:
                actions->add(playFreeKick());
                break;
            default:
                cerr << "[WARNING] Player can not handle this Play Mode!\n";
                actions->add(playPlayOn());
                break;
        }

        //camera motion
        shared_ptr<JointAction> jact(new JointAction(false));
        if (NULL == WM.lastPerception().vision().get()) {
            jact->setForCamera(0, WM.getSearchSpeed().x());
            jact->setForCamera(1, WM.getSearchSpeed().y());
            actions->add(jact);
        } else {
            shared_ptr<CameraMotion> cm(new CameraMotion(mCameraMotionMode));
            actions->add(cm->perform());
        }
        return actions;
    }

    shared_ptr<Action> Player::playKickOff() {
        TTeamIndex ti = WM.getOurTeamIndex();
        TPlayMode pm = WM.getPlayMode();
        if ((TI_LEFT == ti && PM_KICK_OFF_LEFT == pm)
                || (TI_RIGHT == ti && PM_KICK_OFF_RIGHT == pm)) {
            return playOurKickOff();
        } else {
            return playOppKickOff();
        }
    }

    shared_ptr<Action> Player::playKickIn() {
        TTeamIndex ti = WM.getOurTeamIndex();
        TPlayMode pm = WM.getPlayMode();
        if ((TI_LEFT == ti && PM_KICK_IN_LEFT == pm)
                || (TI_RIGHT == ti && PM_KICK_IN_RIGHT == pm)) {
            return playOurKickIn();
        } else {
            return playOppKickIn();
        }
    }

    shared_ptr<Action> Player::playCornerKick() {
        TTeamIndex ti = WM.getOurTeamIndex();
        TPlayMode pm = WM.getPlayMode();
        if ((TI_LEFT == ti && PM_CORNER_KICK_LEFT == pm)
                || (TI_RIGHT == ti && PM_CORNER_KICK_RIGHT == pm)) {
            return playOurCornerKick();
        } else {
            return playOppCornerKick();
        }
    }

    shared_ptr<Action> Player::playGoalKick() {
        TTeamIndex ti = WM.getOurTeamIndex();
        TPlayMode pm = WM.getPlayMode();
        if ((TI_LEFT == ti && PM_GOAL_KICK_LEFT == pm)
                || (TI_RIGHT == ti && PM_GOAL_KICK_RIGHT == pm)) {
            return playOurGoalKick();
        } else {
            return playOppGoalKick();
        }
    }

    shared_ptr<Action> Player::playOffSide() {
        TTeamIndex ti = WM.getOurTeamIndex();
        TPlayMode pm = WM.getPlayMode();
        if ((TI_LEFT == ti && PM_OFFSIDE_LEFT == pm)
                || (TI_RIGHT == ti && PM_OFFSIDE_RIGHT == pm)) {
            return playOurOffSide();
        } else {
            return playOppOffSide();
        }
    }

    shared_ptr<Action> Player::playGoal() {
        TTeamIndex ti = WM.getOurTeamIndex();
        TPlayMode pm = WM.getPlayMode();
        if ((TI_LEFT == ti && PM_GOAL_LEFT == pm)
                || (TI_RIGHT == ti && PM_GOAL_RIGHT == pm)) {
            return playOurGoal();
        } else {
            return playOppGoal();
        }
    }

    shared_ptr<Action> Player::playFreeKick() {
        TTeamIndex ti = WM.getOurTeamIndex();
        TPlayMode pm = WM.getPlayMode();
        if ((TI_LEFT == ti && PM_FREE_KICK_LEFT == pm)
                || (TI_RIGHT == ti && PM_FREE_KICK_RIGHT == pm)) {
            return playOurFreeKick();
        } else {
            return playOppFreeKick();
        }
    }

    shared_ptr<Action> Player::goTo(const Vector2f& destPos, AngDeg bodyDir, bool avoidBall) {
        Vector2f relTarget;
        float turnAng;
        Vector2f destRelPos = WM.transGlobalPosToRelPos(WM.getMyGlobalPos2D(), destPos);
        if (destRelPos.length() > 0.2f) //far enough, so don't care about body direction, just turn to destination
        {
            turnAng = -atan2Deg(destRelPos.x(), destRelPos.y());
            if (fabs(turnAng) > 15.0f)
                relTarget = Vector2f(0, 0);
            else
                relTarget = destRelPos;
        } else {
            relTarget = destRelPos;
            turnAng = normalizeAngle(bodyDir - WM.getMyBodyDirection()); ///////////////////////////////
        }
        return goToAvoidBlocks(relTarget, turnAng, avoidBall);
    }

    shared_ptr<Action> Player::goToRel(const Vector2f& target, AngDeg dir) {
        //first should keep balance
        shared_ptr<Action> act = mBalance.perform();
        if (NULL != act.get()) {
            mKickLock = false;
            mCameraMotionMode = 0;
            mTask.clear();
            return act;
        }

        shared_ptr<Task> walkTask(new WalkRel(target, dir));
        shared_ptr<Task> curTask = mTask.getFirstSubTask();

        //there's no task, append the walk directly
        if (NULL == curTask.get()) {
            mTask.append(walkTask);
            return mTask.perform();
        }

        shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
        if (NULL == curWalkRel.get()) //I am not walking, append the walk
        {
            if (mTask.getSubTaskListSize() <= 1)
                mTask.append(walkTask);
            return mTask.perform();
        } else //I am walking now, revise the current walk
        {
            curWalkRel->revise(walkTask);
            return mTask.perform();
        }
    }

    shared_ptr<Action> Player::goTo(const Vector2f& stopPos, const Vector2f& lookAt, bool avoidBall) {
        Vector2f v = lookAt - stopPos;
        return goTo(stopPos, v.angle(), avoidBall);
    }

    shared_ptr<Action> Player::kickBetween(const Vector2f& goalLeft, const Vector2f& goalRight) {
        const Vector2f& posBall = WM.getBallGlobalPos2D();
        Vector2f vBL = goalLeft - posBall;
        Vector2f vBR = goalRight - posBall;
        const AngDeg angLeft = vBL.angle();
        const AngDeg angRight = vBR.angle();
        const AngDeg angBisector = calBisectorTwoAngles(angRight, angLeft);
        Vector2f goal = posBall + pol2xyz(Vector2f(10.0f, angBisector));

        return kickTo(goal);
    }

    shared_ptr<Action> Player::walkToBall() {
        return goTo(WM.getBallGlobalPos2D(),
                (WM.getBallGlobalPos2D() - WM.getMyOrigin2D()).angle());
    }

    /**
     * just beam to my initial position
     * and turn all joints to 'init' angle
     */
    shared_ptr<Action> Player::beamAndInit(const Vector3f& beamPos) {
        mBalance.perform();
        mBalance.reset();
        mTask.clear();

        shared_ptr<Action> bAct = shared_ptr<Action > (new BeamAction(beamPos));
        shared_ptr<Action> jAct = FAT.controlPreferThan("init", "*");

        Vector2f posMe, beamPos2D;
        posMe = WM.getMyGlobalPos();
        beamPos2D = beamPos;

        shared_ptr<Actions> acts = shared_ptr<Actions > (new Actions);
        if ((beamPos2D - posMe).length() < 0.05) {
            // if we have beam to the given position, stop beaming
            jAct = FAT.controlPreferThan("squat", "*");
            acts->add(jAct);
        } else {
            acts->add(bAct);
            acts->add(jAct);
        }

        return acts;
    }

    shared_ptr<Action> Player::kickTo(const Vector2f& goal, bool useMaxForceMotion) {
        shared_ptr<Action> act;

        //is kicking
        shared_ptr<Task> curTask = mTask.getFirstSubTask();
        shared_ptr<BasicKick> curKick = shared_dynamic_cast<BasicKick > (curTask);
        if (NULL != curKick.get()) {
            act = mTask.perform();
            if (NULL != act.get()) {
                mCameraMotionMode = 0;
                return act;
            } else {
                mKickLock = false;
            }
        }

        //keep balance
        act = mBalance.perform();
        if (NULL != act.get()) {
            mKickLock = false;
            mCameraMotionMode = 0;
            mTask.clear();
            return act;
        }

        //choose kick type and calculate the target of walking
        Vector2f myDesiredPos;
        AngDeg myDesiredBodyDir;
        Vector2f relPosToStopWalk;
        shared_ptr<BasicKick> kick = chooseKickType(goal, useMaxForceMotion, &myDesiredPos, &myDesiredBodyDir, &relPosToStopWalk);

        if ((WM.getBallRelPos2D() + relPosToStopWalk).length() < 0.013f) //kick //0.01f 0.02f
        {
            shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
            if (NULL != curWalkRel.get()) {
                curWalkRel->stopWalk();
                act = mTask.perform();
                if (NULL != act.get()) {
                    return act;
                }
            }
            //	printf("===========%s============\n","new KickTask in kickTo");
            mTask.clear();
            mTask.append(kick);
            return mTask.perform();
        } else if ((myDesiredPos - WM.getMyGlobalPos2D()).length() < 0.07f //0.03f
                && fabs(normalizeAngle(myDesiredBodyDir - WM.getMyBodyDirection())) < 8.0f) //return goToRel
        {
            return goToAvoidBlocks(WM.getBallRelPos2D() + relPosToStopWalk, 0, true);
        } else //return goTo
        {
            return goTo(myDesiredPos, myDesiredBodyDir, true); /////////////////////////////////
        }
    }

    boost::shared_ptr<task::BasicKick> Player::chooseKickType(const math::Vector2f& goal, bool useMaxForceMotion, Vector2f* pPos, AngDeg* pDir, Vector2f* pRelPosToStopWalk) {
        //compare dist and choose a motion
        int i = 0, motionNum = 0; //select motionNum
        float targetDistToGoal = 100.0f;
        float tempFloat;
        Vector2f kickTarget;

        if (useMaxForceMotion) {
            motionNum = 0; //////////////////////////////////// which leg????????????????
        } else {

            FOR_EACH(iter, mKickMotionVector) {
                kickTarget = WM.transRelPosToGlobalPos(WM.getMyGlobalPos2D(), iter->kickTargetRel);
                tempFloat = (kickTarget - goal).length();
                if (tempFloat < targetDistToGoal) {
                    targetDistToGoal = tempFloat;
                    motionNum = i;
                }
                i++;
            }
        }

        //////////////////////////////////////////////
        //motionNum=7;//////////////////////////////////////test

        //calculate pos and dir to go
        const Vector2f& ballPos = WM.getBallGlobalPos2D();
        Vector2f myDesiredPos = WM.transRelPosToGlobalPos(ballPos, mKickMotionVector[motionNum].myDesiredRelPosToBall);
        AngDeg myDesiredBodyDir = (goal - ballPos).angle() + atan2Deg(mKickMotionVector[motionNum].kickTargetRel.x(),
                mKickMotionVector[motionNum].kickTargetRel.y());
        *pPos = myDesiredPos;
        *pDir = myDesiredBodyDir;
        *pRelPosToStopWalk = mKickMotionVector[motionNum].relPosToBallToStopWalk;

        return shared_ptr<KickTask > (new KickTask(mKickMotionVector[motionNum].firstTaskName));
    }



    boost::shared_ptr<action::Action> Player::dribbleToDir(AngDeg angC, AngDeg angL, AngDeg angR) {
        const Vector2f& ballPos = WM.getBallGlobalPos2D();
        const Vector2f& myPos = WM.getMyGlobalPos2D();

        AngDeg myDirToBall = (ballPos - myPos).angle();

        if (isAngInInterval(myDirToBall, angR, angL)) //TT note: angR<angL
        {
            return dribbleRel();
        } else {
            float dist = min(0.3f, WM.getBallPol2D().x()); //distance to ball
            float x = ballPos.x() - dist * cosDeg(angC);
            float y = ballPos.y() - dist * sinDeg(angC);
            return goTo(Vector2f(x, y), angC, true);
        }
    }

    boost::shared_ptr<action::Action> Player::dribble() {
        if (WM.seenFlagsNum() >= 3) {
            const Vector2f& myPos = WM.getMyGlobalPos2D();
            const Vector2f& ballPos = WM.getBallGlobalPos2D();
            Vector2f target(half_field_length, 0);

            //calculate L and R
            Vector2f pointL(half_field_length, half_goal_width);
            Vector2f pointR(half_field_length, -half_goal_width);
            AngDeg angC = (target - ballPos).angle();
            AngDeg angL = (pointL - ballPos).angle();
            AngDeg angR = (pointR - ballPos).angle();

            float ballDistToTarget = (target - ballPos).length();

            if (ballDistToTarget < 4.0f) //near enough to the target
            {
                return dribbleToDir(angC, angL, angR);
            } else {
                //=================enlarge the range
                float k = 0.96f; ///////////////////////////////////// 0.48f
                float deltaDist = ballDistToTarget - 4.0f;
                //L
                float deltaAngL = angL - angC;
                deltaAngL += deltaAngL * k * deltaDist;
                angL = normalizeAngle(angC + deltaAngL);
                //R
                float deltaAngR = angC - angR;
                deltaAngR += deltaAngR * k * deltaDist;
                angR = normalizeAngle(angC - deltaAngR);

                //=================restrict L and R according to the court info
                //...
                return dribbleToDir(angC, angL, angR);
            }
        } else {
            return dribbleToOppGoal();
        }
    }

    shared_ptr<Action> Player::goToAvoidBlocks(math::Vector2f dest, math::AngDeg bodyDir, bool avoidBall) {
        float walkDir = -atan2Deg(dest.x(), dest.y());

        const std::list<core::WorldModel::BlockInfo>& blockList = WM.getBlockList();
        const core::WorldModel::BlockInfo& ballBlock = WM.getBallBlock();

        float nearestPlayerBlockDist = 100.0f; //used in judgement for ball

        //ball
        if (avoidBall && WM.canSeeBall() && ballBlock.dist < nearestPlayerBlockDist) //can see?????????????????
        {
            do {
                if ((ballBlock.dist) > dest.length()/**0.9f*/)
                    break;

                /*||walkDir ballBlock.angC*/

                if (false == isAngInInterval(walkDir, ballBlock.angR, ballBlock.angL))
                    break;

                //the ball blocks me
                walkDir = isAngInInterval(walkDir, ballBlock.angR, ballBlock.angC) ? ballBlock.angR : ballBlock.angL;
                dest.x() = -ballBlock.dist * sinDeg(walkDir);
                dest.y() = ballBlock.dist * cosDeg(walkDir);

            } while (0);
        }

        return goToRel(dest, bodyDir); //bodyDir, instead of walkDir
    }

    shared_ptr<Action> Player::kickRel() {
        shared_ptr<Action> act;

        //is kicking
        shared_ptr<Task> curTask = mTask.getFirstSubTask();
        shared_ptr<BasicKick> curKick = shared_dynamic_cast<BasicKick > (curTask);

        if (NULL != curKick.get()) {
            act = mTask.perform();
            if (NULL != act.get()) {
                mCameraMotionMode = 0;
                return act;
            } else {
                mKickLock = false;
            }
        }

        //keep balance
        act = mBalance.perform();
        if (NULL != act.get()) {
            mKickLock = false;
            mCameraMotionMode = 0;
            mTask.clear();
            return act;
        }

        //get closer to the ball
        Vector2f tempV2f(0, 0);
        float dir = 0.0f;
        const Vector2f& ballRelPos = WM.getBallRelPos2D();
        bool useRightFoot = (ballRelPos.x() > 0);

        if (ballRelPos.y() < 0.01f) //turn body
        {
            if (ballRelPos.x() < 0) dir = 20;
            else dir = -20;
        } else {
            if (ballRelPos.length() > 0.3f)
                dir = (-1) * atan2Deg(ballRelPos.x(), ballRelPos.y());

            if (fabs(dir) < 15.0f)
                tempV2f = ballRelPos + Vector2f((useRightFoot ? -0.07f : +0.07f), -0.19f); //-0.055f -0.19f
        }

        //decide to walk or kick
        if (tempV2f.length() > 0.013f || fabs(dir) > 8.0f) //////////////////////////////////// 0.01f
        {
            act = goToRel(tempV2f, dir);
            return act;
        } else {
            shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
            if (NULL != curWalkRel.get()) {
                curWalkRel->stopWalk();
                act = mTask.perform();
                if (NULL != act.get()) {
                    return act;
                }
            }
            printf("===========%s============\n", "new kick in kickRel");
            shared_ptr<BasicKick> kick = useRightFoot ? shared_ptr<KickTask > (new KickTask("BTTL_t1")) :
                    shared_ptr<KickTask > (new KickTask("BTT_t1"));
            //shared_ptr<BasicKick> kick= shared_ptr<Shoot>( new Shoot(Vector2f(0,0)) );

            //printf("size=%d\n",mTask.getSubTaskListSize()); //1 or sometimes 0
            mTask.clear();
            //printf("size=%d\n",mTask.getSubTaskListSize()); //0
            mTask.append(kick);
            //printf("size=%d\n",mTask.getSubTaskListSize()); //1
            act = mTask.perform();
            //printf("size=%d\n",mTask.getSubTaskListSize()); //should be 1
            return act;
        }
    }


    boost::shared_ptr<action::Action> Player::sideWalk(bool isLeft) {
        if (isLeft) {
            Vector2f tar(-0.15, 0);
            return goToRel(tar, 0);
        } else {
            Vector2f tar(0.15, 0);
            return goToRel(tar, 0);
        }

    }

    shared_ptr<Action> Player::dribbleRel() {
        const Vector2f& ballRelPos = WM.getBallRelPos2D();
        float dir = WM.getBallPol2D().y();

        //for dribble FORWARD
        Vector2f adjustV2f((ballRelPos.x() > 0 ? -0.05f : 0.05f), 0.05f); //////////////////////////////
        //===========

        return goToRel(ballRelPos + adjustV2f, dir);
    }

    shared_ptr<Action> Player::dribbleToOppGoal() {
        mCameraMotionMode = 4; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        float angBall = WM.getBallPol2D().y();
        float angGoal1 = WM.getFlagPol2D(Vision::G1R).y();
        float angGoal2 = WM.getFlagPol2D(Vision::G2R).y();

        //=======================
        Vector2f goalRelPos;
        float ballDistToGoal;
        float k = 0.96f; ////////////////////////////////////////
        float angRange = 0;

        if (WM.canSeeFlag(Vision::G1R) && WM.canSeeFlag(Vision::G2R)) {
            goalRelPos = Vector2f(WM.getFlagRelPos2D(Vision::G1R) + WM.getFlagRelPos2D(Vision::G2R)) / 2;
            ballDistToGoal = (goalRelPos - WM.getBallRelPos2D()).length();
            angRange = k * ballDistToGoal;
            if ((angGoal1 + angRange) > angBall && angBall > (angGoal2 - angRange))
                return dribbleRel();
            else
                return goToBallBack();
        } else if (WM.seenFlagsNum() >= 3 && WM.canSeeBall()) //use global info
        {
            Vector2f deltaV2f = Vector2f(half_field_length, 0) - WM.getMyGlobalPos2D();
            float angG = deltaV2f.angle() - WM.getMyBodyDirection();
            float myDistToGoal = deltaV2f.length();
            float xg = -myDistToGoal * sinDeg(angG);
            float yg = myDistToGoal * cosDeg(angG);
            goalRelPos = Vector2f(xg, yg);

            ballDistToGoal = (goalRelPos - WM.getBallRelPos2D()).length();
            angRange = k * ballDistToGoal;
            if ((angGoal1 + angRange) > angBall && angBall > (angGoal2 - angRange))
                return dribbleRel();
            else
                return goToBallBack();
        } else if (WM.canSeeFlag(Vision::G1R)) {
            goalRelPos = WM.getFlagRelPos2D(Vision::G1R);
            ballDistToGoal = (goalRelPos - WM.getBallRelPos2D()).length();
            angRange = k * ballDistToGoal;
            if ((angGoal1 + angRange) > angBall) ////////////////////////////////////////
                return dribbleRel();
            else
                return goToBallBack();
        } else if (WM.canSeeFlag(Vision::G2R)) {
            goalRelPos = WM.getFlagRelPos2D(Vision::G2R);
            ballDistToGoal = (goalRelPos - WM.getBallRelPos2D()).length();
            angRange = k * ballDistToGoal;
            if (angBall > (angGoal2 - angRange)) ////////////////////////////////////////
                return dribbleRel();
            else
                return goToBallBack();
        } else {
            return goToBallBack();
        }
    }

    shared_ptr<Action> Player::goToBallBack() {
        mCameraMotionMode = 4; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        float xb = WM.getBallRelPos2D().x();
        float yb = WM.getBallRelPos2D().y();
        float angB = WM.getBallPol2D().y();
        float myDistToBall = WM.getBallPol2D().x();
        float xg = 0, yg = 0, angG = 0;
        float x = 0, y = 0, ang = 0;

        //========================info
        if (WM.canSeeFlag(Vision::G1R) && WM.canSeeFlag(Vision::G2R)) {
            xg = (WM.getFlagRelPos2D(Vision::G1R).x() + WM.getFlagRelPos2D(Vision::G2R).x()) / 2;
            yg = (WM.getFlagRelPos2D(Vision::G1R).y() + WM.getFlagRelPos2D(Vision::G2R).y()) / 2;
            angG = -atan2Deg(xg, yg); //TT: this is accurate
        } else if (WM.seenFlagsNum() >= 3 && WM.canSeeBall()) {
            //don't want to use global position
            Vector2f deltaV2f = Vector2f(half_field_length, 0) - WM.getMyGlobalPos2D();
            angG = deltaV2f.angle() - WM.getMyBodyDirection();
            float myDistToGoal = deltaV2f.length();
            xg = -myDistToGoal * sinDeg(angG);
            yg = myDistToGoal * cosDeg(angG);
        } else if (WM.canSeeFlag(Vision::G1R)) {
            xg = WM.getFlagRelPos2D(Vision::G1R).x();
            yg = WM.getFlagRelPos2D(Vision::G1R).y();
            angG = WM.getFlagPol2D(Vision::G1R).y();
        } else if (WM.canSeeFlag(Vision::G2R)) {
            xg = WM.getFlagRelPos2D(Vision::G2R).x();
            yg = WM.getFlagRelPos2D(Vision::G2R).y();
            angG = WM.getFlagPol2D(Vision::G2R).y();
        } else {
            //mCameraMotionMode=3; //search flags
            return goToRel(WM.getBallRelPos2D(), -atan2Deg(xb, yb)); /////////////////////////////////////
        }


        //======================================
        //get closer to the ball
        if (myDistToBall > 1.5f) {
            float D = min(0.7f, myDistToBall); //desired distance to ball
            float d = sqrt((xb - xg)*(xb - xg) + (yb - yg)*(yb - yg));
            if (d < EPSILON) d = EPSILON;

            x = (xb - xg) * D / d + xb;
            y = (yb - yg) * D / d + yb;
            ang = -atan2Deg(x, y);

            if (ang > 90.0f) ang -= 180.0f;
            else if (ang<-90.0f) ang += 180.0f;

            if (fabs(ang) < 30.0f)
                return goToAvoidBlocks(Vector2f(x, y), ang, true);
            else
                return goToRel(Vector2f(0, 0), ang);
        } else {
            float D = min(0.3f, myDistToBall); //desired distance to ball
            float d = sqrt((xb - xg)*(xb - xg) + (yb - yg)*(yb - yg));
            if (d < EPSILON) d = EPSILON;

            x = (xb - xg) * D / d + xb;
            y = (yb - yg) * D / d + yb;
            ang = angG;

            return goToAvoidBlocks(Vector2f(x, y), ang, true);
        }
    }

    shared_ptr<Action> Player::fallToGetBall(int dir) {
        shared_ptr<Action> act;
        shared_ptr<Task> curTask = mTask.getFirstSubTask();
        shared_ptr<BasicKick> curKick = shared_dynamic_cast<BasicKick > (curTask);

        if (NULL != curKick.get()) {
            act = mTask.perform();

            if (NULL != act.get()) {
                mCameraMotionMode = 0;
                return act;
            } else {
                ;
            }
        }

        //keep balance
        act = mBalance.perform();
        if (NULL != act.get()) {
            mKickLock = false;
            mCameraMotionMode = 0;
            mTask.clear();
            return act;
        }

        //new fall
        shared_ptr<WalkRel> curWalkRel = shared_dynamic_cast<WalkRel > (curTask);
        if (NULL != curWalkRel.get()) {
            curWalkRel->stopWalk();
            act = mTask.perform();
            if (NULL != act.get()) {
                return act;
            }
        }
        if (-1 == dir) {
            shared_ptr<BasicKick> kick = shared_ptr<KickTask > (new KickTask("leftfall_pt_init_squat"));
            mTask.clear();
            mTask.append(kick);
            act = mTask.perform();
            return act;
        } else if (1 == dir) {
            shared_ptr<BasicKick> kick = shared_ptr<KickTask > (new KickTask("rightfall_pt_init_squat"));
            mTask.clear();
            mTask.append(kick);
            act = mTask.perform();
            return act;
        } else if (-2 == dir) {
            shared_ptr<BasicKick> kick = shared_ptr<KickTask > (new KickTask("LFToLie1_wcy"));
            mTask.clear();
            mTask.append(kick);
            act = mTask.perform();
            return act;
        } else if (2 == dir) {
            shared_ptr<BasicKick> kick = shared_ptr<KickTask > (new KickTask("RFToLie1_wcy"));
            mTask.clear();
            mTask.append(kick);
            act = mTask.perform();
            return act;
        }
    }
} // namespace soccer