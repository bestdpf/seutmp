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


        BEGIN_ADD_LOG_LAYER(TeamPlayer)
        ADD_LOG_LAYER("decision")
        ADD_LOG_LAYER("deadball")
        ADD_LOG_LAYER("defense")
        ADD_LOG_LAYER("terrymimi")
        ADD_LOG_LAYER("vision")
        ADD_LOG_LAYER("vision-me")
        ADD_LOG_LAYER("deal-pos")
        ADD_LOG_LAYER("jia")
        ADD_LOG_LAYER("StickToBall")
        ADD_LOG_LAYER("cylinder")
        ADD_LOG_LAYER("searchBall")
        ADD_LOG_LAYER("corporation")
        ADD_LOG_LAYER("see")
        ADD_LOG_LAYER("specialDribble")
        ADD_LOG_LAYER("test")
        ADD_LOG_LAYER("TT")
        END_ADD_LOG_LAYER(TeamPlayer)
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
        // return goTo(Vector2f(8,0),0);
        // return clearBall();
        // return dribble(Vector2f(6,0));
        // return walkToBall();
        // return kickTo(Vector2f(-6,0));

        if (WM.isGameStateChanged())
            mTask.clear();


        shared_ptr<Action> act;
        // cout << "eeeeeeeee" << FM.getMy().type<<"wwwww"<<WM.getMyUnum() << "\n";
        //     cout <<"@"<<WM.getGameTime() <<"fastest" <<WM.getOurFastestToBallNum()<<endl;
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER: //No.1
                act = goalKeeperBehaviour();
                break;
            case configuration::Formation::PT_ATTACKER_CENTRAL: //No.9
                act = attackerCentralBehaviour();
                break;
            case configuration::Formation::PT_ATTACKER_WING: //No.8
                act = attackerLeftWingBehaviour();
                //   act = attackerBehaviour();
                //   act = defaultBehaviour();
                break;
            case configuration::Formation::PT_MIDFIELDER_WING: //No.7
                act = attackerRightWingBehaviour();
                //   act = attackerBehaviour();
                //   act = defaultBehaviour();
                break;
            case configuration::Formation::PT_MIDFIELDER_CENTER: //No.6
                act = middleFielderBehaviour();
                // act = defaultBehaviour();
                break;
            case configuration::Formation::PT_MIDFIELDER_SWEEPER://No.5
                act = defenderSweeperBehaviour();
                // act = defaultBehaviour();
                break;
            case configuration::Formation::PT_DEFENDER_CENTRAL: //No.2
                act = defenderCenteralBehaviour();
                break;
            case configuration::Formation::PT_DEFENDER_SWEEPER://No.4
                act = defenderLeftWingBehaviour();
                break;
            case configuration::Formation::PT_DEFENDER_WING: //No.3
                act = defenderRightWingBehaviour();
                //  act = defaultBehaviour();
                break;
            default:
                act = defaultBehaviour();
                break;
        }

        LOG_FLUSH;
        return act;
    }

    /** before kick off */
    shared_ptr<Action> TeamPlayer::playBeforeKickOff() { ///terrymimi
        //  cout << "fast"<<WM.getOurFastestToBallNum()<<endl;
        return beamAndInit(FM.getMy().beforeKickOffBeam + Vector3f(-0.2, 0.3, 0));
    }

    /** kick off */
    shared_ptr<Action> TeamPlayer::playOurKickOff() {
        //   cout << "fast"<<WM.getOurFastestToBallNum()<<endl;
        shared_ptr<Action> act;
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f runtarget;
        switch (FM.getMy().type) {
            case configuration::Formation::PT_ATTACKER_CENTRAL:
            {
                runtarget.x() = 3;
                runtarget.y() = 3;
                //Vector2f runtargetRel = GlobalToRel(runtarget);
                act = kickTo(runtarget);
                break;
            }
//                case configuration::Formation::PT_ATTACKER_WING:
//	    {
//		runtarget.x() = -0.2;
//		runtarget.y() = -0.5;
//		act = goTo(runtarget,Vector2f(0,1).angle());
//		break;
//
//	    }
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
        Vector2f BallPos=WM.getBallGlobalPos2D();
        float delta1=0.3;
        float delta2=0.3;
        if(BallPos.y()>0)
        {
            if(FM.getMy().type==configuration::Formation::PT_MIDFIELDER_SWEEPER)//No.5
            {
                Vector2f MyPos=WM.getMyGlobalPos2D();
                Vector2f destination;
                destination.x()=BallPos.x()-delta1;
                if((MyPos.x()<BallPos.x())&&(MyPos.y()>BallPos.y()))
                {
                    destination.y()=BallPos.y()+delta2;
                    return runStrategicPosnew(destination);
                }
                else
                {
                    Vector2f PassTarget=PM.getOurTeammates()[6].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(6);
                    PassTarget.x()=PassTarget.x()+1;
                    return kickTo(PassTarget);
                }

            }
            if(FM.getMy().type==configuration::Formation::PT_DEFENDER_SWEEPER)//No.6
            {
                Vector2f destination;
                destination.x()=BallPos.x();
                destination.y()=half_field_width/2;
                return runStrategicPosnew(destination);
            }
        }
        else
        {
            if(FM.getMy().type==configuration::Formation::PT_DEFENDER_SWEEPER)//No.6
            {
                Vector2f MyPos=WM.getMyGlobalPos2D();
                Vector2f destination;
                destination.x()=BallPos.x()-delta1;
                if((MyPos.x()<BallPos.x())&&(MyPos.y()<BallPos.y()))
                {
                    destination.y()=BallPos.y()-delta2;
                    return runStrategicPosnew(destination);
                }
                else
                {
                    Vector2f PassTarget=PM.getOurTeammates()[5].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(5);
                    PassTarget.x()=PassTarget.x()+1;
                    return kickTo(PassTarget);
                }

            }
            if(FM.getMy().type==configuration::Formation::PT_MIDFIELDER_SWEEPER)//No.5
            {
                Vector2f destination;
                destination.x()=BallPos.x();
                destination.y()=-half_field_width/2;
                return runStrategicPosnew(destination);
            }

        }
        return playOurDeadBall();
    }

    shared_ptr<Action> TeamPlayer::playOppKickIn() {
        return playOppDeadBall();
    }

    /** corner kick */
    shared_ptr<Action> TeamPlayer::playOurCornerKick() {
        shared_ptr<Action> act;
        float delta1=0.3;
        float delta2=0.3;
        Vector2f BallPos = WM.getBallGlobalPos2D();
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER:
                act = goTo(calGoalKeeperDefensePos(), WM.getBallGlobalPos2D());
                break;
            case configuration::Formation::FT_ATTACK_LEFT:
            {
                if(BallPos.y()>0)
                {
                    Vector2f MyPos=WM.getMyGlobalPos2D();
                    Vector2f destination;
                    destination.x()=BallPos.x()+delta1;
                    if((MyPos.x()>BallPos.x())&&(MyPos.y()<BallPos.y()))
                    {
                        destination.y()=BallPos.y()-delta2;
                        return runStrategicPosnew(destination);
                    }
                    else
                    {
                        Vector2f PassTarget=PM.getOurTeammates()[9].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(9);
                        PassTarget.x()=PassTarget.x()+1;
                        return kickTo(PassTarget);
                    }

                }
                else
                {
                    Vector2f destination;
                    destination.x()=half_field_length-penalty_length/2+2;
                    destination.y()=0;
                    return runStrategicPosnew(destination);
                }
            }
            case configuration::Formation::FT_ATTACK_RIGHT:
            {
                if(BallPos.y()<=0)
                {
                    Vector2f MyPos=WM.getMyGlobalPos2D();
                    Vector2f destination;
                    destination.x()=BallPos.x()+delta1;
                    if((MyPos.x()>BallPos.x())&&(MyPos.y()<BallPos.y()))
                    {
                        destination.y()=BallPos.y()-delta2;
                        return runStrategicPosnew(destination);
                    }
                    else
                    {
                        Vector2f PassTarget=PM.getOurTeammates()[9].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(9);
                        PassTarget.x()=PassTarget.x()+1;
                        return kickTo(PassTarget);
                    }

                }
                else
                {
                    Vector2f destination;
                    destination.x()=half_field_length-penalty_length/2+2;
                    destination.y()=0;
                    return runStrategicPosnew(destination);
                }
            }
            case configuration::Formation::FT_ATTACK_MIDDLE:
            {
                Vector2f destination;
                destination.x()=half_field_length-penalty_length/2+3;
                if(BallPos.y()>0)
                    destination.y()=half_field_length/2;
                else
                    destination.y()=-half_field_length/2;
                return runStrategicPosnew(destination);

            }
            default:
                act = playPlayOn();
                break;
        }

        LOG_FLUSH;
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
                const Vector2f& myPos=WM.getMyGlobalPos2D();
                const Vector2f& ballPos=WM.getBallGlobalPos2D();
                Vector2f Destination(-2, myPos.y()>0 ? -2 : 2 );
                Vector2f ourGoal(-half_field_length,0);
                
                if( myPos.x() < ballPos.x() )
                {
                    return kickTo(Destination,true);
                }
                else
                {
                    return goTo(ourGoal,0,true);
                }
                break;
            }
            case configuration::Formation::PT_DEFENDER_SWEEPER://No.4
            {
                    Vector2f Destination(-half_field_length+3,-1);
                    return runStrategicPosnew(Destination);

            }
            case configuration::Formation::PT_DEFENDER_CENTRAL://No.2
            {

                    Vector2f Destination(-half_field_length+4,0);
                    return runStrategicPosnew(Destination);


            }
            case configuration::Formation::PT_DEFENDER_WING: //No.3
            {

                    Vector2f Destination(-half_field_length+3,1);
                    return runStrategicPosnew(Destination);
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
        // struggle pose here ;-/
    }

    /** free kick */
    shared_ptr<Action> TeamPlayer::playOurFreeKick() {
        return playOurDeadBall();
    }

    shared_ptr<Action> TeamPlayer::playOppFreeKick() {
        return playOurDeadBall();
    }


    ///////////////////////////////////////////////////////////

    shared_ptr<Action> TeamPlayer::runStrategicPos() {

        Vector3f strategicPos =
                FM.calMyStrategicPos(WM.getBallGlobalPos());
        Vector2f posStop(strategicPos.x(), strategicPos.y());
        //cout << "ball" << WM.getBallGlobalPos2D() << endl;
        //cout << "me" << WM.getMyGlobalPos2D() << endl;
        //cout << posStop << endl;
        // return goTo(posStop, WM.getBallGlobalPos2D() - posStop);
        math::AngDeg dir = WM.getMyBodyDirection();
        Vector2f tar(posStop - WM.getMyGlobalPos2D());
        Vector2f x(1, 0);
        dir = dir + calClipAng(tar, x);
        return goToRel(posStop - WM.getMyGlobalPos2D(), dir);
    }

    shared_ptr<Action> TeamPlayer::runOpenPos() {
        unsigned int num = WM.getMyUnum();
        const Vector2f& posMe = WM.getMyOrigin2D();

        const Vector2f& posBall = WM.getBallGlobalPos2D();
        float pass_dist = 5;
        float minX = max(-half_field_length, posBall.x() - pass_dist);
        minX = min(minX, posMe.x() - 0.5f);
        float maxX = min(half_field_length, posBall.x() + pass_dist);
        float minY = max(-half_field_width, posBall.y() - pass_dist);
        float maxY = min(half_field_width, posBall.y() + pass_dist);

        const Vector2f plb(minX, minY);
        const Vector2f prb(maxX, minY);
        const Vector2f prt(maxX, maxY);
        const Vector2f plt(minX, maxY);

        // the polygon of the soccer field
        TVoronoi<float> voronoi;
        voronoi.create(plb, prb, prt, plt);

        if (!voronoi.setInnerPoint(posMe)) {
            LOG_RED_LINE_2D("decision", plb, prb);
            LOG_RED_LINE_2D("decision", prb, prt);
            LOG_RED_LINE_2D("decision", prt, plt);
            LOG_RED_LINE_2D("decision", plt, plb);
            LOG_PRINT("decision", "I am out side, but this should not happen!!");
            return runStrategicPos();
        }

        Vector2f p;
        const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();

        FOR_EACH(iter, ourPos) {
            if (iter->first == num) continue;
            p.x() = iter->second.x();
            p.y() = iter->second.y();
            Line2f line = Line2f::makeMidperpendicularFromTwoPoints(p, posMe);
            voronoi.cutByLine(line);
        }

        const map<unsigned int, Vector3f>& oppPos = WM.getOppGlobalPos();

        FOR_EACH(iter, oppPos) {
            p.x() = iter->second.x();
            p.y() = iter->second.y();
            Line2f line = Line2f::makeMidperpendicularFromTwoPoints(p, posMe);
            voronoi.cutByLine(line);
        }

        p = voronoi.calCentroid();
        AngDeg ang2ball = (posBall - posMe).angle();
        AngDeg ang2goal = (Vector2f(half_field_length, 0) - posMe).angle();
        AngDeg ang = (ang2goal + ang2ball)*0.5f;
        return goTo(p, ang);
    }

    shared_ptr<Action> TeamPlayer::defaultBehaviour() {
        /**
         * the simplest decision:
         * If I am the fastes, shoot the ball,
         * otherwise run to the strategic position
         */
        //    WM.setTurnCameraSign(true);
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
        WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f myPos = WM.getMyGlobalPos2D();


        float distToBall = (ballPos - myPos).length();
        float ballDistToOppGoal = (oppGoal - ballPos).length();
        float ballAngToOppGoal = (oppGoal - ballPos).angle();
        // if(distToBall<5){
        if (WM.getHearOurFastestToBallNum() == WM.getMyUnum()) {
            //if ( WM.amIFastestToBallOfOurTeam() ){
            //cout << "Shoot" <<endl;
            return kickTo(oppGoal);
            //return pass();
            // Vector3f goal(9,0,0);
            // return kickTo(Vector2f(6,0),true);
        } else {
            // cout << "runStrategicPos" << endl;
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

 shared_ptr<Action> TeamPlayer::changeRole(int onum) {
        shared_ptr<Action> act;
        switch (onum) {
            case 1: //No.1
                act = goalKeeperBehaviour();
                break;
            case 9: //No.9
                act = attackerCentralBehaviour();
                break;
            case 8: //No.8
                act = attackerLeftWingBehaviour();
                //   act = attackerBehaviour();
                //   act = defaultBehaviour();
                break;
            case 7: //No.7
                act = attackerRightWingBehaviour();
                //   act = attackerBehaviour();
                //   act = defaultBehaviour();
                break;
            case 6: //No.6
                act = middleFielderBehaviour();
                // act = defaultBehaviour();
                break;
            case 5://No.5
                act = defenderSweeperBehaviour();
                // act = defaultBehaviour();
                break;
            case 2: //No.2
                //   cout << "ddd";
                act = defenderCenteralBehaviour();
                break;
            case 4://No.4
                act = defenderLeftWingBehaviour();
                break;
            case 3: //No.3
                act = defenderRightWingBehaviour();
                //  act = defaultBehaviour();
                break;
            default:
                act = defaultBehaviour();
                break;
        }
        return act;
    }

     ////////////////////////////////////////////////////////////////////////////////////
/***********************************************Gravity Penalty Test***********************************************************************/
   shared_ptr<action::Action>TeamPlayer::playoffenderpenalty() {
        Vector2f goal(half_field_length, 0);
        Vector2f goooal1(penalty_length, -half_penalty_width);
        Vector2f goooal2(penalty_length, half_penalty_width);
        //Vector2f goal2(half_field_length, -2);
        const Vector2f& ballPos = WM.getBallGlobalPos2D();
        float xBall = ballPos[0];
        float yBall = ballPos[1];
        if(xBall<=2.0f)return kickTo(goal,true);
        else{
        if (xBall < 0.68 * half_field_length||xBall>0.9*half_field_length) {
           // if (yBall < 0)
                return dribble();
            //else
              //  return dribble(goooal2);
        } else {

            return kickTo(goal,true);
        }
        }
    }

 shared_ptr<action::Action>TeamPlayer::playdefenderpenalty() {
        //return goTo(Vector2f(-0.55*half_field_length,0),0,true);
        Vector2f goal(half_field_length, 0);
        Vector2f oppgoal(-half_field_length, 0);
        Vector2f cleargoal1(0,-half_field_width);
        Vector2f cleargoal2(0,half_field_width);

         Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f myPos = WM.getMyGlobalPos2D();
        Vector2f oppPos =WM.getOppGlobalPos2D(6);
        //float xPenalty = 0.2*half_field_length;
        //float yPenalty = 0.5*half_field_width;
        Vector2f dPos;
        Vector2f penaltyright(penalty_length - half_field_length, -half_penalty_width);
        Vector2f penaltyleft(penalty_length - half_field_length, half_penalty_width);
        float xBall = ballPos[0];
        float yBall = ballPos[1];
        float xMe = myPos[0];
        float yMe = myPos[1];
        float xOpp = oppPos[0];
        float yOpp = oppPos[1];
        //float xdPos;
        //float ydPos;
        //float distToBall = (ballPos - myPos).length();
        //float maxX = penalty_length;
        //float minY1 = half_penalty_width;
        //float minY2 = -half_penalty_width;
        Vector2f goalLeft(-half_field_length, half_goal_width);
        Vector2f goalRight(-half_field_length, -half_goal_width);
        const Vector2f& posBall = WM.getBallGlobalPos2D();
        Vector3f ballVel = WM.getBallGlobalVel();
        Vector2f ballVel2D(ballVel.x(), ballVel.y());
        float ballV0=ballVel2D[0];
        float ballV1=ballVel2D[1];

        float distL = (goalLeft - posBall).length();
        float distR = (goalRight - posBall).length();
        float dist = distL > distR ? distR : distL;
        //AngDeg angL = (goalLeft - posBall).angle();
        //AngDeg angR = (goalRight - posBall).angle();
        AngDeg ang = (posBall - oppgoal).angle();
        AngDeg ang1 = ang+90;//calClipAng(angL, angR)*0.5f;
        AngDeg angLBoder = (penaltyleft - oppgoal).angle();
        AngDeg angRBoder = (penaltyright - oppgoal).angle();
        //AngDeg dir = calBisectorTwoAngles(angL, angR);
        dist *= cosDeg(ang);
        dist -= 0.5f;
        //Vector2f p = posBall + pol2xyz(Vector2f(dist, dir));
        //p.y() = clamp(p.y(), -half_goal_width + 0.055f, half_goal_width - 0.055f);
        // return p;
        //cout<<"love me"<<endl;
        /*if (abs(ang) < abs(angLBoder)) {
            dPos[0] = 0.5*(penalty_length - half_field_length);
            //cout<<"dPOS0"<<dPos[0]<<endl;
            dPos[1] =0.5*( yBall * penalty_length / (xBall + half_field_length));
            //cout<<"dPOS1"<<dPos[1]<<endl;

        } else {
            if (yBall > 0) {
                dPos[0] = 0.5*((xBall + half_field_length)*half_penalty_width/yBall - half_field_length);
                //cout<<"dPOS 0"<<dPos[0]<<endl;
                dPos[1] = 0.5*half_penalty_width;
                //cout<<"dPOS 1"<<dPos[1]<<endl;
            } else {
                dPos[0] = 0.5*((xBall + half_field_length)*half_penalty_width/abs(yBall) - half_field_length);
                //cout<<"dPOS"<<dPos[0];
                dPos[1] = 0.5*(-half_penalty_width);
                //cout<<"dPOS"<<dPos[1];
            }
        }
        //return goTo(dPos,0,true);
        */
        dPos[0]=-half_field_length+0.5f;
        dPos[1]=yBall/(half_field_length+xBall);
        if (WM.isItInOurPenaltyZone(ballPos) ){
                if (xOpp > xMe) {
                    return dribble();
                } else{
                    return kickTo(goal,true);
                }
        }
        else {
           // if ((xBall < -0.5 * half_field_length)) {
     /*           if (WM.isItInOurPenaltyZone(myPos)) {

                    reurn goTo(dPos, ang ,true);
                } else {
                    return goTo(oppgoal);

            }
                  else
                   reurn goToRel(dPos);
    */
                //if( ! WM.isItInOurPenaltyZone(myPos)) return goTo(oppgoal,ang,true);
               // else
                    //if(xBall > -0.4*half_field_length)
                       // return goTo(dPos,ang1,true);
                    //else return keepGoal();
                    //dPos[0]=-half_field_length+1.0f;
                    //dPos[1]=yBall/(half_field_length+xBall);

            if(xMe+2.2f>=xBall)//||abs(yMe)+1.5f>=abs(yBall))
                        return penaltyKiller();
            else
                return goTo(dPos,ang,false);
              //  }
             // if(xBall > -0.55*half_field_length)
             // return goTo(dPos,ang1,true);
              //else return keepGoal();


    }
    }
shared_ptr<action::Action>TeamPlayer::playGK() {
    Vector2f ballPos = WM.getBallGlobalPos2D();
    Vector2f myPos = WM.getMyGlobalPos2D();
    Vector2f dPos;
    float xBall = ballPos[0];
    float yBall = ballPos[1];
    dPos[0]=-half_field_length+0.5f;
    dPos[1]=yBall/(half_field_length+xBall);
    float xMe = myPos[0];
    float yMe = myPos[1];
    //float xOpp = oppPos[0];
    //float yOpp = oppPos[1];
    Vector2f oppPos =WM.getOppGlobalPos2D(6);
    Vector2f goal(half_field_length, 0);
    Vector2f oppgoal(-half_field_length, 0);
    AngDeg ang = (ballPos - oppgoal).angle();
    if (WM.amIFastestToBallOfOurTeam()){
                //if (xOpp > xMe) {
                    return dribble();
                //} else{
                   // return kickTo(goal,true);
                //}
        }
        else {
            return goTo(dPos,ang,false);
        }
}

 /****************************************************Gravity***********************************************************************/



    ////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Action> TeamPlayer::goalKeeperBehaviour() //No.1
{
	//mCameraMotionMode=2; //TT: stick to ball !!!

	const Vector2f& myPos=WM.getMyGlobalPos2D();
	Vector2f goalPos(-half_field_length-1.0f,0); /////////////////////////////
	const Vector2f& ballPos=WM.getBallGlobalPos2D();
	float myDistToBall=WM.getBallPol2D().x();

	float xg=goalPos.x(), yg=goalPos.y();
	float xb=ballPos.x(), yb=ballPos.y();

	//===================common pos
	float deltaX=0.33f; //0.17f
	float x=-half_field_length+deltaX;
	float y=yb*deltaX/(xb-xg);

	Vector2f myDesiredPos(x,y);
	//AngDeg myDesiredBodyDir= yb>0 ? 90.0f : -90.0f ;
	AngDeg myDesiredBodyDir=0;


	//==========================vel
	const Vector2f& ballRelPos=WM.getBallRelPos2D();
	Vector2f ballRelVel(0,0);

	static float lastSeeBallTime=0;
	static Vector2f lastBallRelPos(WM.getBallRelPos2D());
	static Vector2f lastBallRelVel(0,0);

	bool amIInPZone= fabs(myPos.y())<half_penalty_width-0.2f
						&& myPos.x()>-half_field_length-0.7f
						&& myPos.x()<-half_field_length+penalty_length-0.2f ;

	if( NULL!=WM.lastPerception().vision().get() && WM.canSeeBall() && amIInPZone )
	{
		LOG_PRINTF("GK","[@%.2f], ballRelPos(%.1f, %.1f)",WM.getGameTime(),ballRelPos.x(),ballRelPos.y());
		float timeAfterLastSeeBall=WM.getGameTime()-lastSeeBallTime;
		if(timeAfterLastSeeBall<0.35f) //vision cycle is 0.06s //////////////////////////////
		{
			//calculate myDesiredPos
			ballRelVel= (ballRelPos-lastBallRelPos) / timeAfterLastSeeBall ;
			LOG_PRINTF("GK","dist: %.2f,   vel: %.2f",myDistToBall,ballRelVel.length());

			if( (  (myDistToBall<5.0f && ballRelVel.length()>4.4f && lastBallRelVel.length()>4.5f)
				|| (myDistToBall<4.0f && ballRelVel.length()>4.3f && lastBallRelVel.length()>4.4f)
				|| (myDistToBall<3.0f && ballRelVel.length()>3.3f && lastBallRelVel.length()>3.4f)
				|| (myDistToBall<2.0f && ballRelVel.length()>3.2f && lastBallRelVel.length()>3.3f)
				)
			 && (ballRelVel.y()<0 && lastBallRelVel.y()<0) )
			{
				//calculate the cross point on X-axis
				float xbr=ballRelPos.x(), ybr=ballRelPos.y();
				float xr=xbr-ybr*ballRelVel.x()/ballRelVel.y();

				if( fabs(xr)<0.3f ){ //0.3f
					return fallToGetBall( xr>0 ? 1 : -1 );
				}
				else if( fabs(xr)<1.0f ){
					return fallToGetBall( xr>0 ? 2 : -2 );
				}
				else if( fabs(xr)<1.5f ){
					return goToRel( Vector2f(xr,0) , 0 );
				}

			}

		}

		lastSeeBallTime=WM.getGameTime();
		lastBallRelPos=ballRelPos;
		lastBallRelVel=ballRelVel;
	}


	//special situation
	if( WM.amIFastestToBallOfOurTeam() && myDistToBall<2.5f && amIInPZone ) ////////////////
	{
		return clearBall();
	}


	return goTo(myDesiredPos,myDesiredBodyDir,false);
}


    //both hear and see

    bool TeamPlayer::isMeFastestToBall() {
        //     if (0 == WM.getHearOurFastestToBallNum()) {
        //     cout << "Hear error" << endl;
        return WM.amIFastestToBallOfOurTeam();
        //  } else {
        //   return WM.getHearOurFastestToBallNum() == WM.getMyUnum();
        //  }
    }
    //both hear and see

    int TeamPlayer::getFastestPlayer() {
        // if (0 == WM.getHearOurFastestToBallNum()) {
        //      cout << "Hear error too" << endl;
        return WM.getOurFastestToBallNum();
        // } else {
        //    return WM.getHearOurFastestToBallNum();
        //  }
    }


////////////////////////////////////////////
///////////////////////////////////////////
    bool TeamPlayer::suitableForShoot()
    {
    	Vector2f MyPos=WM.getMyGlobalPos2D();
    	bool condition= (MyPos.x()>(half_field_length-3.5))&&(MyPos.y()>half_field_width/2)&&(MyPos.y()<-half_field_width/2);
    	if(condition) return true;
        else return false;

    }


	bool TeamPlayer::suitableForPass()
	{
            int threshold=1.5;
            float toMeDistance=WM.getOppClosestToMeDistance();
            if(toMeDistance>1.5*field_length)
                toMeDistance=1.5*field_length;
            if(toMeDistance<0)
                toMeDistance=1;
            bool condition=toMeDistance>threshold;
            if(condition)
                return true;
            else
                return false;

	}

        bool TeamPlayer::suitableForDribble()
        {
            int OneInFront=WM.FindOneInFrontMe();
            if(OneInFront>9||OneInFront<1)
                return true;
            bool condition=(OneInFront==-1);
            if(condition)
                return true;
            else
                return false;

        }
    shared_ptr<Action> TeamPlayer::runStrategicPosnew(Vector2f & target)
    {
        //Vector3f BallPos=WM.getBallGlobalPos();
        Vector2f ballPos = WM.getBallGlobalPos2D();
       // math::AngDeg dir=WM.getMyFaceDirection();
        if(target.x() >= 10) target.x() = 10;
        if(target.y() >= 7)  target.y() = 7;
        if(target.y() <= -7) target.y() = -7;
        return goTo(target, ballPos,1);
    }


    shared_ptr<Action> TeamPlayer::attack()
    {
       // Vector2f oppDoor(0,-half_field_length);
        return dribble(/*GlobalToRel(oppDoor)*/);
    }


/////////////////////////////////////////////////////

   shared_ptr<Action> TeamPlayer::attackerCentralBehaviour() //No.9
    {

    	WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f destination[2];
	Vector2f myPos = WM.getMyGlobalPos2D();
	Vector2f ballPos = WM.getBallGlobalPos2D();
	Vector2f oppGoal(half_field_length,0);
        destination[0] =PM.getOurTeammates()[8].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(8);
        destination[1] =PM.getOurTeammates()[7].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(7);//
             //cout<<"38uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu"<<endl;
//        if (WM.getOurPlayerNumber() <= 1 && WM.getOppPlayerNumber() <= 1){
//        return playoffenderpenalty();
//        }
      //  if(0)
        if(WM.amIFastestToBallOfOurTeam())
        {
	  //if(myPos.x() > destination[0].x() && myPos.y() > destination[1].x())
	//  {
	    if(ballPos.x() > 6.5) return shoot();
	    else return dribble();
	//  }
          if(WM.GetOppCount(destination[0])>WM.GetOppCount(destination[1]))
		return kickTo(destination[1]);
	  else
	 	return kickTo(destination[0]);

          if(suitableForShoot())
        	    return kickTo(oppGoal);
          if(suitableForDribble())
                    return dribble();

        }
       else
        {
            Vector2f BallPos=WM.getBallGlobalPos2D();
            //caculate my destination
            /*if(BallPos.x()>0)
            {
                Vector2f Pos_8=PM.getOurTeammates()[8].getMGlobalPosAll2D();;
                Vector2f Pos_7=PM.getOurTeammates()[7].getMGlobalPosAll2D();;
                Vector2f Minus=Pos_8-Pos_7;
                Vector2f ConvertMinus;
                ConvertMinus.x()=Minus.y();
                ConvertMinus.y()=-Minus.x();
                ConvertMinus.x()=ConvertMinus.x()/ConvertMinus.length();
                ConvertMinus.y()=ConvertMinus.y()/ConvertMinus.length();
                Vector2f Destination;
                if(ConvertMinus.x()>0)
                    Destination=(Pos_8+Pos_7)/2+Minus.length()*(ConvertMinus);
                else
                    Destination=(Pos_8+Pos_7)/2-Minus.length()*(ConvertMinus);
                if((Destination.x()>half_field_length)||(Destination.x()<-half_field_length)||(Destination.y()>half_field_width)||(Destination.y()<-half_field_width))
                {
                    ConvertMinus.x()=-Minus.y();
                    ConvertMinus.y()=Minus.x();
                    ConvertMinus.x()=ConvertMinus.x()/ConvertMinus.length();
                    ConvertMinus.y()=ConvertMinus.y()/ConvertMinus.length();
                    //Vector2f Destination;
                    if(ConvertMinus.x()>0)
                        Destination=(Pos_8+Pos_7)/2+Minus.length()*(ConvertMinus);
                    else
                        Destination=(Pos_8+Pos_7)/2-Minus.length()*(ConvertMinus);
                }

                return runStrategicPosnew(Destination);
            }
            else
            {*/

                Vector2f Destination;
                int increment=2.5;
                Destination.x()=BallPos.x();
                if(BallPos.y()>0)
                {
                    Destination.y()=BallPos.y()-increment;
                }
                else
                {
                    Destination.y()=BallPos.y()+increment;
                }
                return runStrategicPosnew(Destination);
            //}
        }
    }
   shared_ptr<Action> TeamPlayer::attackerLeftWingBehaviour()//No.8
    {
        WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f BallPos=WM.getBallGlobalPos2D();
        Vector2f destination(half_field_length/2,0);
        Vector2f oppGoal(half_field_length,0);
        if(WM.amIFastestToBallOfOurTeam())
        {
	    if(BallPos.x() > 6.5) return shoot();
	    else return dribble();

        }
        else
        {
            if(BallPos.y()>0)
            {
                destination.y()=BallPos.y()-2;
                if(BallPos.x()>2*half_field_length/3)
                {
                    destination.x()=BallPos.x()-2.5;
                }
                else
                {
                    destination.x()=BallPos.x()+2;
                }
            }
            else
            {
                destination.y()=BallPos.y()+2;
                if(BallPos.x()>2*half_field_length/3)
                {
                    destination.x()=BallPos.x()-2.5;
                }
                else
                {
                    destination.x()=BallPos.x()+2;
                }
            }
            return runStrategicPosnew(destination);
        }

    }







   /*shared_ptr<Action> TeamPlayer::attackerLeftWingBehaviour()//No.8
    {
        WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f destination[2];
	Vector2f myPos = WM.getMyGlobalPos2D();
	Vector2f ballPos = WM.getBallGlobalPos2D();
	Vector2f oppGoal(half_field_length,0);

        destination[0] =PM.getOurTeammates()[7].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(7);
        destination[1] =PM.getOurTeammates()[9].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(9);
        //if(0)
       if(WM.amIFastestToBallOfOurTeam())
        {
        	if(suitableForShoot())
        		return kickTo(oppGoal);
                if(suitableForDribble())
                    return dribble();
		else{
		  	if(myPos.x() > destination[0].x() && myPos.y() > destination[1].x())
			    {
			      if(ballPos.x() > 6.5) return kickTo(oppGoal);
			      else return dribble();
			    }
			if(WM.GetOppCount(destination[0])>WM.GetOppCount(destination[1]))
		        	return kickTo(destination[1]);
		        else
		        	return kickTo(destination[0]);
		}

        }
        else
        {
            float LengthOfAdjacentPerson=3.5;
            //caculate my destination
            Vector2f Pos_9=PM.getOurTeammates()[9].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(9);
            Vector2f Pos_7=PM.getOurTeammates()[7].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(7);
            Vector2f Destination;
            Vector2f BallPos=WM.getBallGlobalStopPos2D();
            float Ball29=(BallPos-Pos_9).length();
            float Ball27=(BallPos-Pos_7).length();
            if(Ball29>Ball27)
            {
                Destination.y()=Pos_7.y();
                if(Pos_7.x()>0)
                {
                    Destination.x()=Pos_7.x()-LengthOfAdjacentPerson;
                }
                else
                {
                    Destination.x()=Pos_7.x()+LengthOfAdjacentPerson;
                }
            }
            else
            {
                Vector2f Pos_7=PM.getOurTeammates()[7].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(7);
                Vector2f Pos_9=PM.getOurTeammates()[9].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(9);
                Vector2f Minus=Pos_7-Pos_9;
                Vector2f ConvertMinus;
                ConvertMinus.x()=Minus.y();
                ConvertMinus.y()=-Minus.x();
                ConvertMinus.x()=ConvertMinus.x()/ConvertMinus.length();
                ConvertMinus.y()=ConvertMinus.y()/ConvertMinus.length();
                Vector2f Destination;
                if(ConvertMinus.x()>0)
                    Destination=(Pos_9+Pos_7)/2+Minus.length()*(ConvertMinus);
                else
                    Destination=(Pos_9+Pos_7)/2-Minus.length()*(ConvertMinus);
                if((Destination.x()>half_field_length)||(Destination.x()<-half_field_length)||(Destination.y()>half_field_width)||(Destination.y()<-half_field_width))
                {
                    ConvertMinus.x()=-Minus.y();
                    ConvertMinus.y()=Minus.x();
                    ConvertMinus.x()=ConvertMinus.x()/ConvertMinus.length();
                    ConvertMinus.y()=ConvertMinus.y()/ConvertMinus.length();
                    //Vector2f Destination;
                    if(ConvertMinus.x()>0)
                        Destination=(Pos_9+Pos_7)/2+Minus.length()*(ConvertMinus);
                    else
                        Destination=(Pos_9+Pos_7)/2-Minus.length()*(ConvertMinus);
                }

            }
            //caculate my destination

            return runStrategicPosnew(Destination);

        }

    }
    */


    shared_ptr<Action> TeamPlayer::attackerRightWingBehaviour() //No.7
    {
        WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f destination[2];
	Vector2f myPos = WM.getMyGlobalPos2D();
	Vector2f ballPos = WM.getBallGlobalPos2D();
	Vector2f oppGoal(half_field_length,0);
        destination[0] =PM.getOurTeammates()[8].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(8);
        destination[1] =PM.getOurTeammates()[9].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(9);
        if(WM.amIFastestToBallOfOurTeam())
        {
            if(ballPos.x() > 6.5) return shoot();
	    else return dribble();
        }
        else
        {
            float LengthOfAdjacentPerson=3.5;
            //caculate my destination
            Vector2f Pos_8=PM.getOurTeammates()[8].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(8);
            Vector2f Pos_9=PM.getOurTeammates()[9].getMGlobalPosAll2D();//WM.getOurGlobalPos2D(9);
            Vector2f Destination;
            Vector2f BallPos=WM.getBallGlobalStopPos2D();
            float Ball28=(BallPos-Pos_8).length();
            float Ball29=(BallPos-Pos_9).length();
            if(Ball28>Ball29)
            {
                Destination.y()=Pos_9.y();
                if(Pos_9.x()>0)
                {
                    Destination.x()=Pos_9.x()-LengthOfAdjacentPerson;
                }
                else
                {
                    Destination.x()=Pos_9.x()+LengthOfAdjacentPerson;
                }
            }
            else
            {
                Destination.y()=Pos_8.y();
                if(Pos_8.x()>0)
                {
                    Destination.x()=Pos_8.x()-LengthOfAdjacentPerson;
                }
                else
                {
                    Destination.x()=Pos_8.x()+LengthOfAdjacentPerson;
                }

            }
            return runStrategicPosnew(Destination);


        }

    }
 ////////////////////////////////////////////////////
 ///////////////////////////////////////////////////
    shared_ptr<Action> TeamPlayer::defenderSweeperBehaviour()//No.5
    {
        Vector2f oppGoal(half_field_length, 0);
	Vector2f ballPos = WM.getBallGlobalPos2D();
        if(WM.amIFastestToBallOfOurTeam())
        {
            if(ballPos.x()>0)
            {
                if(ballPos.x() > 6.5) return shoot();
                else return dribble();
            }
            else
            {
                float incre=0;
                if(ballPos.x()>0)
                {
                    incre=-2;
                }
                else
                {
                    incre=2;
                }
                Vector2f destination(oppGoal.x(),oppGoal.y()+incre);
                return kickTo(oppGoal,false);
            }

        }
        else
        {
            Vector2f OurDoor(-half_field_length,0);
            Vector2f BallPos=WM.getBallGlobalPos2D();
            Vector2f Destination(-half_field_length/2,half_field_width/2);
            float alpha=0.1;
            Destination=BallPos+alpha*(OurDoor-BallPos);
            //Destination.y()=Destination.y()+1.5;
            //float threshold;
            //float radius=1;
            //Vector2f Destination;
            //Vector2f BallPos=WM.getBallGlobalPos2D();
            //Vector2f MyPos=WM.getMyGlobalPos2D();
            //if((BallPos-MyPos).length()<radius)
            //{
                //threshold=1;
                //Destination.x()=BallPos.x()-radius;
                //Destination.y()=BallPos.y()+threshold/2;
            //}
            //else
            //{
                //threshold=2;
                //Destination.x()=BallPos.x()-radius;
                //Destination.y()=BallPos.y()+threshold/2;
            //}
            return runStrategicPosnew(Destination);
        }
    }

    shared_ptr<Action> TeamPlayer::middleFielderBehaviour()//No.6
    {
        Vector2f oppGoal(half_field_length, 0);
	Vector2f ballPos = WM.getBallGlobalPos2D();

        if(WM.amIFastestToBallOfOurTeam())
        {
	    if(ballPos.x()>0)
            {
                if(ballPos.x() > 6.5) return shoot();
                else return dribble();
            }
            else
            {
                float incre=0;
                if(ballPos.x()>0)
                {
                    incre=-2;
                }
                else
                {
                    incre=2;
                }
                Vector2f destination(oppGoal.x(),oppGoal.y()+incre);
                return kickTo(oppGoal,false);
            }

        }
        else
        {
            Vector2f OurDoor(-half_field_length,0);
            Vector2f BallPos=WM.getBallGlobalPos2D();
            Vector2f Destination(-half_field_length/2,-half_field_width/2);
            float alpha=0.3;
            Destination=BallPos+alpha*(OurDoor-BallPos);
            //Destination.y()=Destination.y()-3;
            //float threshold;
            //float radius=1;
            //Vector2f Destination;
            //Vector2f BallPos=WM.getBallGlobalPos2D();
            //Vector2f MyPos=WM.getMyGlobalPos2D();
            //if((BallPos-MyPos).length()<radius)
            //{
                //threshold=1;
                //Destination.x()=BallPos.x()-radius;
                //Destination.y()=BallPos.y()-threshold/2;
            //}
            //else
            //{
                //threshold=2;
                //Destination.x()=BallPos.x()-radius;
                //Destination.y()=BallPos.y()-threshold/2;
            //}
            return runStrategicPosnew(Destination);
        }

    }




//////////////////////////////////////////////////////////////////////////////////////////
unsigned int TeamPlayer::numNearBall()
{
	const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();
        Vector2f myPos = WM.getMyGlobalPos2D();
        Vector2f ballPos = WM.getBallGlobalPos2D();
	int num;
	float minDis=0;
	float temp=0;
        FOR_EACH(iter, ourPos) {
		switch(iter->first)
		{
			case 2:
				if(iter->second.x()< ballPos.x()){
					minDis=sqrt(pow(ballPos.x(),iter->second.x())+pow(ballPos.y(),iter->second.y()));
					temp=minDis;
					num=iter->first;
				}
				break;
			case 3:
			case 4:
				if(iter->second.x()< ballPos.x()){
					minDis=sqrt(pow(ballPos.x(),iter->second.x())+pow(ballPos.y(),iter->second.y()));
					if(minDis<temp) {
						temp=minDis;
						num=iter->first;
					}
				}
				break;
		}
		}
	if(temp != 0) return num;
	else return 1;
}




    bool TeamPlayer::isBallInMyZone(Vector2f ballPos)
    {
      if(ballPos.x() <= -5.0) return true;
      else return false;
    }

    bool TeamPlayer::isBallInMyZoneT(Vector2f ballPos)
    {
      if(ballPos.x() <= -6.5) return true;
      else return false;
    }
    bool TeamPlayer::isBallInMyZoneTT(Vector2f ballPos)
    {
      if(ballPos.x() <= -7.2) return true;
      else return false;
    }
    shared_ptr<Action> TeamPlayer::defenderRightWingBehaviour() //No.4
    {

        WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f myPos = WM.getMyGlobalPos2D();
        Vector2f oppGoal(half_field_length, 0);
        Vector2f runtarget(ballPos.x() - 1, ballPos.y() - 0.5);
        float distToBall = (ballPos - myPos).length();
        float ballDistToOppGoal = (oppGoal - ballPos).length();
        float ballAngToOppGoal = (oppGoal - ballPos).angle();
        runtarget = Vector2f(ballPos.x() - 1.5,ballPos.y()*(ballPos.x()-1.5+half_field_length)/(ballPos.x()+half_field_length) );
	if(isMeFastestToBall())
	{
	  if(isBallInMyZoneT(ballPos))
	  return dribble();
	  else return kickTo(oppGoal,false);
	}
	if(isBallInMyZone(ballPos))
	{
	 if(WM.getMyUnum() == numNearBall()){
	  if(isBallInMyZoneTT(ballPos)) return dribble();
	  return dribble();

	  }
	 else{
	 if(runtarget.x() >= -9.4) return goTo(runtarget,ballPos,0);
	 else return goTo(Vector2f(-10.6,ballPos.y() + 1),ballPos,0);
	    }
	}
		else return goTo(Vector2f(-5.0,ballPos.y()),ballPos,0);
    }

    shared_ptr<Action> TeamPlayer::defenderCenteralBehaviour() //No.2
    {

        WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f myPos = WM.getMyGlobalPos2D();
        Vector2f oppGoal(half_field_length, 0);
        Vector2f runtarget(ballPos.x() - 1, ballPos.y() + 0.5);
        runtarget = Vector2f(ballPos.x() - 1.5,ballPos.y()*(ballPos.x()-1.5+half_field_length)/(ballPos.x()+half_field_length) );
	if(3 == numNearBall()) {
	  runtarget.x() = ballPos.x() - 1.8;
	  runtarget.y() = ballPos.y()*(ballPos.x()-1.8+half_field_length)/(ballPos.x()+half_field_length );
	}
	else runtarget.x() = ballPos.x() - 1.5;
        float distToBall = (ballPos - myPos).length();
        float ballDistToOppGoal = (oppGoal - ballPos).length();
        float ballAngToOppGoal = (oppGoal - ballPos).angle();
	if(isMeFastestToBall())
	{
	  if(isBallInMyZoneT(ballPos))
	  return dribble();
	  else return kickTo(oppGoal,false);
	}
	if(isBallInMyZone(ballPos))
	{
	  if(WM.getMyUnum() == numNearBall()){
	    if(isBallInMyZoneTT(ballPos)) return dribble();
	    return dribble();
	  }
	  else{
	    if(runtarget.x() >= -9.4) return goTo(runtarget,ballPos,0);
	    else return goTo(Vector2f(-10.6,ballPos.y() + 1),ballPos,0);
	  }
	}
	else return goTo(Vector2f(-6.0,ballPos.y() - 1),ballPos,0);

    }

    shared_ptr<Action> TeamPlayer::defenderLeftWingBehaviour() //No.3
    {

        WM.setLocalWalkSign(false);
        WM.setRushToGoalSign(false);
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f myPos = WM.getMyGlobalPos2D();
        Vector2f oppGoal(half_field_length, 0);
        Vector2f runtarget(ballPos.x() - 2, ballPos.y() + 1);
        float distToBall = (ballPos - myPos).length();
        float ballDistToOppGoal = (oppGoal - ballPos).length();
        float ballAngToOppGoal = (oppGoal - ballPos).angle();
        runtarget = Vector2f(ballPos.x() - 1.8,ballPos.y()*(ballPos.x()-1.8+half_field_length)/(ballPos.x()+half_field_length));
	if(isMeFastestToBall())
	{
	  if(isBallInMyZoneT(ballPos))
	  return dribble();
	  else return kickTo(oppGoal,false);
	}
	if(isBallInMyZone(ballPos))
	{
	  if(WM.getMyUnum() == numNearBall()){
	    if(isBallInMyZoneTT(ballPos)) return dribble();
	    return dribble();
	  }
	  else{
	    if(runtarget.x() >= -9.4) return goTo(runtarget,ballPos,0);
	    else return goTo(Vector2f(-10.6,ballPos.y() + 1),ballPos,0);
	  }
	}
	else return goTo(Vector2f(-6.0,ballPos.y() + 1),ballPos,0);

    }

    //////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////

    shared_ptr<Action> TeamPlayer::playOurDeadBall() {
        if (WM.getPerceptions()[WM.getPerceptions().size() - 2]->gameState().getPlayMode() == PM_PLAY_ON) {
            mIsPassing = false;
        }

        shared_ptr<Action> act;
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER:
                act = goTo(calGoalKeeperDefensePos(), WM.getBallGlobalPos2D());
                break;

            default:
                act = playPlayOn();
                break;
        }

        LOG_FLUSH;
        return act;
    }

    shared_ptr<Action> TeamPlayer::playOppDeadBall() {
        shared_ptr<Action> act;
        switch (FM.getMy().type) {
            case configuration::Formation::PT_GOALKEEPER:
            {
                LOG_PRINT("deadball", "run goal keeper defense position");
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
                if (isMeFastestToBall()) {
                    // if (WM.amIFastestToBallOfOurTeam()) {
                    LOG_PRINT("deadball", "block opp kick");
                    const Vector2f& posBall = WM.getBallGlobalPos2D();
                    const Vector2f goal(-half_field_length, 0);
                    AngDeg ang = (goal - posBall).angle();
                    Vector2f p = pol2xyz(Vector2f(free_kick_distance + 0.5f, ang)) + posBall;
                    return goTo(p, posBall);
                } else {
                    LOG_PRINT("deadball", "run defense position");
                    // act = runDefensePos();
                    act = playPlayOn();
                }
                break;
            }
        }

        LOG_FLUSH;
        return act;
    }

    bool TeamPlayer::isDefenseStable() {
        const Vector2f& posBall = WM.getBallGlobalPos2D();
        // if ( posBall.x() > 0 ){
        //     LOG_PRINT("defense","ball is on opponent's half field");
        //     return true;
        // }

        float minX = -half_field_length;
        float maxX = posBall.x();
        float minY = -abs(posBall.y());
        float maxY = -minY;

        int defenseNum = 0;
        const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();

        FOR_EACH(iter, ourPos) {
            const Vector3f& p = iter->second;
            if (p.x() > minX && p.x() < maxX && p.y() > minY && p.y() < maxY) {
                LOG_PRINTF("defense", "teammate %d is defensing", iter->first);
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

    boost::shared_ptr<action::Action> TeamPlayer::defenseShoot() {
        if (WM.getBallRelVel2D().length() < 0.2) {
            Vector2f leftBlock(-10.5f, 1.5f); //
            Vector2f rightBlock(-10.5f, -1.5f);
            Vector2f tar(GlobalToRel(calDefensePos(leftBlock, rightBlock)));
            return goToRel(tar, WM.getBallRelPos2D().angle() - 90.0f);
        } else {
            return interceptionBall();
        }
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
