/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include <list>
#include <boost/bind.hpp>
#include "AStar.hpp"
#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "MoveCoM.h"
#include "MoveFoot.h"
#include "StepSlow.h"
#include "WalkSlow.h"


namespace task{
    using namespace std;
    using namespace boost;
    using namespace math;
    using namespace serversetting;
    using namespace action;

    float WalkSlow::mWalkHeight = 0.35f;

    DEFINE_STATIC_GRAPHIC_LOGGER(WalkSlow)

    WalkSlow::WalkSlow(const Vector2f& target, AngDeg direction, bool avoidBall, Task* primary)
    :Task(-1, primary), // -1: as soon as possible
        mTarget(target),
        mDirection(direction),
        mSizeErrorThreshold(0.03f,0.03f),
        mDirErrorThreshold(3.0f),
        mIsWalking(false),
        mAvoidBall(avoidBall)
    {
        BEGIN_ADD_STATIC_LOG_LAYER(WalkSlow)
            ADD_LOG_LAYER("newWalk")
            ADD_LOG_LAYER("newStep")
            ADD_LOG_LAYER("planPath")
            ADD_LOG_LAYER("blocks")
        END_ADD_STATIC_LOG_LAYER(WalkSlow)

            LOG_PRINTF("newWalk","new a walk (%.3f, %3.f; %3.f)",
                       mTarget.x(),mTarget.y(),direction);

        mPreSize.zero();
    }

    bool WalkSlow::isDone() const
    {
        if (!mIsWalking) return false;

        return mSubTaskList.empty();
    }

    bool WalkSlow::revise( shared_ptr<const Task> rt )
    {
        shared_ptr<const WalkSlow> wrt =
            shared_dynamic_cast<const WalkSlow>(rt);
        if ( NULL != wrt.get() ){
            // accept another WalkSlow
            mTarget = wrt->mTarget;
            mDirection = wrt->mDirection;
            mSizeErrorThreshold = wrt->mSizeErrorThreshold;
            mDirErrorThreshold = wrt->mDirErrorThreshold;
            mAvoidBall = wrt->mAvoidBall;
            return true;
        }

        return false;
    }

    void WalkSlow::updateSubTaskList()
    {
        LOG_PRINT("newStep","updateSubTaskList");
        Task::updateSubTaskList();

        if ( isSubTaskOfAllLessThanTwo() ){
            // create the sequence tasks for start walking
            if (!mIsWalking){
                // 1. move CoM
                Vector3f com(HUMANOID.getHalfFeetWidth()*0,
                             /*StepSlow::getMaxSizeAcc().y()*MoveFoot::getPitchRatio().y()*0.5f,*/0,
                             mWalkHeight);
                for( int i=0; i<2; i++ ){
                    shared_ptr<Task> mcom ( new MoveCoM(com, 10*WM.getAverageStepTime(), this ) );
                    mSubTaskList.push_back(mcom);
                }
                mIsWalking = true;
            }

            AngDeg currentDir = WM.getMyBodyDirection();
            Vector2f size = mTarget - WM.getMyOrigin2D();
            AngDeg aiming = mDirection;
            AngDeg dir = calClipAng( aiming, currentDir );

            AngDeg preDir = 0;
            bool isLeft = true;
            shared_ptr<const StepSlow> cStep;
            if ( !mSubTaskList.empty() ){
                cStep = shared_dynamic_cast<const StepSlow>( mSubTaskList.front() );
                if ( 0 != cStep.get() ){
                    isLeft = !cStep->isLeft(); /**< change foot */
                    mPreSize = cStep->size();
                    preDir  = cStep->dir();
                }
            }

            // see if we have reached the target
            if ( fabs(size.x()) < mSizeErrorThreshold.x()
                 && fabs(size.y()) < mSizeErrorThreshold.y()
                 && fabs(dir) < mDirErrorThreshold
                 && fabs(mPreSize.x()) < mSizeErrorThreshold.x()
                 && fabs(mPreSize.y()) < mSizeErrorThreshold.y()
                 && fabs(preDir) < mDirErrorThreshold){
                mSubTaskList.clear();
                LOG_PRINT("newStep","reach the target!");

                LOG_FLUSH;
                return;
            }

            // plan the path
            // -------------------------------
            planPath();
            //
            //mPlanTarget = mTarget;
            //mPlanDir = mDirection;
            // -------------------------------

            size = mPlanTarget - WM.getMyOrigin2D();
            aiming = mPlanDir;
            dir = calClipAng( aiming, currentDir );

            Vector2f polar = xyz2pol(size);
            if ( fabs(calClipAng(polar[1], aiming))/polar[0]
                 < StepSlow::getMaxDir()/StepSlow::getMaxSize().length() ){
                // I am far from the target
                // redirect the aime to the target
                LOG_PRINT("newStep","far away from the target");
                aiming = polar[1];
                dir = calClipAng( aiming, currentDir );

                // back/forward
                if ( fabs(dir) > 90 ){
                    if ( fabs(dir) > 120 || (isLeft && dir<0) || (!isLeft && dir>0) ){
                        dir = normalizeAngle( dir + 180 );
                        LOG_PRINT("newStep","backwards");
                    }
                }

                if ( fabs(dir) > StepSlow::getMaxSize().angle() ){
                    // turning the direction firstly
                    LOG_PRINT("newStep","stop for turning");
                    polar[0] = 0;
                }
            }

            polar[1] = calClipAng( polar[1], currentDir-90 );
            size = pol2xyz(polar);

            LOG_PRINTF("newStep","size length = %.3f",size.length());

            // if the size and dir if bigger than accerlation
            // discount the size the dir
            size *= 0.5f; // NOTE: the real step size is double 'size'
            const float size_k = 1.0f;//0.25f;
            const float dir_k = 1.0f;//0.25f;
            if ( fabs(size.x()) > StepSlow::getMaxSizeAcc().x()
                 || fabs(mPreSize.x()) > StepSlow::getMaxSizeAcc().x() ){
                size.x() *= size_k;
            }
            if ( fabs(size.y()) > StepSlow::getMaxSizeAcc().y()
                 || fabs(mPreSize.y()) > StepSlow::getMaxSizeAcc().y() ){
                size.y() *= size_k;
            }
            if ( fabs(dir) > StepSlow::getMaxDirAcc()
                 || fabs(preDir) > StepSlow::getMaxDirAcc() ){
                dir *= dir_k;
            }

            LOG_PRINTF("newStep","append StepSlow: %.3f %.3f %.3f",
                       size.x(), size.y(), dir);

            // size = Vector2f(0,1);
            // dir = 0;
            shared_ptr<Task> mstep
                ( new StepSlow(isLeft, size, dir, cStep, mWalkHeight, this ) );
            mSubTaskList.push_back(mstep);
        }

        LOG_FLUSH;
    }

    void WalkSlow::generatePossibleState( const State& start, const State& target,
                                      const State& final,
                                      std::vector<State>& res,
                                      std::vector<State>& unReach) const
    {
        std::vector<State>::iterator iter = res.begin();
        for(; iter!=res.end(); ++iter){
            if ( iter->sameAs(target) ){
                    break;
            }
        }
        if ( iter != res.end() ){
            //cout<<"target ("<<target.p()<<")is in the result"<<endl;
            return;
        }

        iter = unReach.begin();
        for(; iter!=unReach.end(); ++iter){
            if ( iter->sameAs(target) ){
                    break;
            }
        }
        if ( iter != unReach.end() ){
            //cout<<"target ("<<target.p()<<")is in the unReach"<<endl;
            return;
        }

        const Vector2f& startPos = start.p();
        const Vector2f& targetPos = target.p();
        const Vector2f& finalPos = final.p();
        //cout<<"generatePossibleState ["<<startPos<<" ("<<start.costed()
        //    <<' '<<start.moreCost()<<")] ==> ["
        //    <<targetPos<<" ("<<target.costed()<<' '<<target.moreCost()<<")]"
        //    <<endl;

        Segment2f originalPath(startPos, targetPos);
        bool isBreak = false;
        FOR_EACH(iter, mBlocks){
            if ( iter->isIntersect(originalPath) ){
                //LOG_RED_LINE_2D("planPath",iter->p0(), iter->p1());
                const Vector2f& p0 = iter->p0();
                const Vector2f& p1 = iter->p1();
                State s0(p0, (p0-startPos).length()+start.costed(),
                         (p0-finalPos).length() );
                State s1(p1, (p1-startPos).length()+start.costed(),
                         (p1-finalPos).length() );

                if ( (s0.sameAs(start) && !s1.sameAs(target))
                     || (s1.sameAs(start) && !s0.sameAs(target))
                     || (s0.sameAs(target) && !s1.sameAs(start))
                     || (s1.sameAs(target) && !s0.sameAs(start))){
                    //cout<<"the start or target is on the line"<<endl;
                    continue;
                }

                if ( !isBreak ){
                    isBreak = true;
                    //cout<<"push ("<<target.p()<<" into unReach"<<endl;
                    unReach.push_back(target);
                }

                //cout<<"find : ["<<p0<<", "<<p1<<"]"<<endl;


                generatePossibleState(start,s0,final,res,unReach);
                generatePossibleState(start,s1,final,res,unReach);
            }
        }

        if ( !isBreak ) {
            // there is not any intersection
            //cout<<"push the ("<<target.p()<<") in results"<<endl;
            res.push_back(target);
        }
    }


    void WalkSlow::planPath()
    {
        //cout<<"start path plan @ "<<WM.getSimTime()<<endl;

        mPlanTarget = mTarget;
        mPlanDir = mDirection;

        const Vector2f& myPos = WM.getMyOrigin2D();

        LOG_BLUE_LINE_2D("planPath",myPos, mTarget);

        float minCost = (myPos-mTarget).length();
        State start(myPos,0,minCost );
        State target(mTarget,minCost,0);

        setupBlocks();

        function<void (const State&, const State&, const State&, std::vector<State>&, std::vector<State>&)> genNext = bind(&WalkSlow::generatePossibleState, this, _1, _2, _3, _4, _5);
        std::list<State> path = AStar(start, target, genNext);

        LOG_PRINTF("planPath","path size: %d",path.size());
        FOR_EACH(iter,path){
            LOG_GREEN_SPHERE("planPath",
                             Vector3f(iter->p().x(),iter->p().y(),0),0.05f);
        }

        if ( path.size()>1 ){
            std::list<State>::const_iterator iter = path.begin();
            mPlanTarget = iter->p();
            iter++;
            Vector2f nextTarget = iter->p();
            mPlanDir = (nextTarget - mPlanTarget).angle();
            mPlanTarget += (mPlanTarget - myPos).normalized();
            LOG_GREEN_SPHERE("planPath",
                             Vector3f(mPlanTarget.x(),mPlanTarget.y(),0),0.1f);
        }

#ifdef ENABLE_LOG
        path.push_front(start);
        FOR_EACH(iter,path){
            list<State>::const_iterator iter1 = iter;
            iter1++;
            if ( iter1 != path.end() ){
                LOG_GREEN_LINE_2D("planPath",iter->p(),iter1->p());
            }
        }
#endif // ENABLE_LOG
    }

    void WalkSlow::setupBlocks()
    {
        mBlocks.clear();

        // Segment2f block0(Vector2f(-1,-0.5),Vector2f(-1.0,0.5));
        // Segment2f block1(Vector2f(-1.5,0),Vector2f(-1.0,-1.5));
        // Segment2f block2(Vector2f(-0.5,-0.5),Vector2f(-1.0,1.0));
        // mBlocks.push_back(block0);
        // mBlocks.push_back(block1);
        // mBlocks.push_back(block2);

        const static float safe_dist = 0.2f;
        const Vector3f& bodySize = HUMANOID.getBoundBoxSize();
        const float self_foot_width = bodySize.x() * 0.5f + 0.1f;
        const float self_body_width = bodySize.x() * 0.5f + safe_dist;
        const float player_body_width = bodySize.x() * 0.5f;
        const float player_body_height = bodySize.z() * 0.5f;
        const float player_fall_height = player_body_height * 0.5f;

        // avoid to collide with the goals
        const static float goal_back_x = half_field_length + goal_depth;
        // left goal
        /**
         *
         *  \|       1    |
         * --+------------+--
         *   |\           |5
         *   |
         *   |
         *   |
         *   |
         *  2|
         *   |
         *   |
         *   |
         *   |
         *   |/           |
         * --+------------+--
         *  /|       3    |4
         */
        const static Vector2f p0(-half_field_length,half_goal_width);
        const static Vector2f p1(-goal_back_x,half_goal_width);
        const static Vector2f p2(-goal_back_x,-half_goal_width);
        const static Vector2f p3(-half_field_length,-half_goal_width);

        const static float bias = self_body_width;
        const static Segment2f leftGoal1(p0+Vector2f(bias,0),
                                         p1+Vector2f(-bias,0));
        const static Segment2f leftGoal2(p1+Vector2f(0,bias),
                                         p2+Vector2f(0,-bias));
        const static Segment2f leftGoal3(p2+Vector2f(-bias,0),
                                         p3+Vector2f(bias,0));
        // check the possibility of colliding the left goal
        Segment2f directWalk(mTarget,WM.getMyOrigin2D());
        const static Segment2f leftGoalFront(
            Vector2f(-half_field_length, goal_width),
            Vector2f(-half_field_length, -goal_width));
        const static Segment2f leftGoalBack(
            Vector2f(-half_field_length, goal_width),
            Vector2f(-half_field_length, -goal_width));
        const static Segment2f leftGoalCenter(
            Vector2f(-half_field_length, 0),
            Vector2f(-goal_back_x, 0));
        if ( leftGoalFront.isIntersect(directWalk)
             || leftGoalBack.isIntersect(directWalk)
             || leftGoalCenter.isIntersect(directWalk) ){
            mBlocks.push_back(leftGoal1);
            mBlocks.push_back(leftGoal2);
            mBlocks.push_back(leftGoal3);
            mBlocks.push_back(buildBlock(p0,self_body_width));
            mBlocks.push_back(buildBlock(p1,self_body_width));
            mBlocks.push_back(buildBlock(p2,self_body_width));
            mBlocks.push_back(buildBlock(p3,self_body_width));
        }

        // right goal
        const static Segment2f rightGoal1(-leftGoal1.p0(),
                                          -leftGoal1.p1());
        const static Segment2f rightGoal2(-leftGoal2.p0(),
                                          -leftGoal2.p1());
        const static Segment2f rightGoal3(-leftGoal3.p0(),
                                          -leftGoal3.p1());
        const static Segment2f rightGoalFront(
            Vector2f(half_field_length, goal_width),
            Vector2f(half_field_length, -goal_width));
        const static Segment2f rightGoalBack(
            Vector2f(half_field_length, goal_width),
            Vector2f(half_field_length, -goal_width));
        const static Segment2f rightGoalCenter(
            Vector2f(goal_back_x, 0),
            Vector2f(half_field_length, 0));
        if ( rightGoalFront.isIntersect(directWalk)
             || rightGoalBack.isIntersect(directWalk)
             || rightGoalCenter.isIntersect(directWalk)){
            mBlocks.push_back(rightGoal1);
            mBlocks.push_back(rightGoal2);
            mBlocks.push_back(rightGoal3);
            mBlocks.push_back(buildBlock(rightGoal1.p0(),self_body_width));
            mBlocks.push_back(buildBlock(rightGoal1.p1(),self_body_width));
            mBlocks.push_back(buildBlock(rightGoal3.p0(),self_body_width));
            mBlocks.push_back(buildBlock(rightGoal3.p1(),self_body_width));
        }

        if (mAvoidBall){
            // avoid to collide with the ball
            const float ball_avoid_radius = self_foot_width + ball_radius;
            const Vector2f& posBall = WM.getInterceptBallGlobalPos2D();//WM.getBallGlobalPos2D();
            mBlocks.push_back(buildBlock(posBall, ball_avoid_radius));
        }

        // avoid to collide with opponents
        const float fallen_player_avoid_radius = self_body_width + player_body_height;
        const float stand_player_avoid_radius = self_body_width + player_body_width;
        const map<unsigned int, Vector3f>& oppPos = WM.getOppGlobalPos();
        FOR_EACH(i, oppPos){
            const Vector3f& pos = i->second;
            if ( pos.z() < player_fall_height ){
                // avoid the fallen opponent
                mBlocks.push_back(buildBlock(Vector2f(pos.x(),pos.y()), fallen_player_avoid_radius));
                LOG_PRINTF("blocks","opponent %d fall down",i->first);
            }
        }

        // avoid to collide with teammates
        unsigned int myNum = WM.getMyUnum();
        const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();
        FOR_EACH(i, ourPos){
            if ( i->first == myNum ) continue;
            const Vector3f& pos = i->second;
            if ( pos.z() < player_fall_height ){
                // avoid the teammates
                mBlocks.push_back(buildBlock(Vector2f(pos.x(),pos.y()), fallen_player_avoid_radius));
                LOG_PRINTF("blocks","teammate %d fall down",i->first);
            }
            else{
                mBlocks.push_back(buildBlock(Vector2f(pos.x(),pos.y()), stand_player_avoid_radius));
                LOG_PRINTF("blocks","teammate %d stand up",i->first);
            }
        }

        // log
        FOR_EACH(iter,mBlocks){
            LOG_RED_LINE_2D("blocks",iter->p0(),iter->p1());
        }
    }

    Segment2f WalkSlow::buildBlock(const Vector2f& p, float radius) const
    {
        // AngDeg ang = (p-mTarget).angle();
        AngDeg ang = mDirection;
        float avoid_x = sinDeg(ang)*radius;
        float avoid_y = cosDeg(ang)*radius;
        return Segment2f(Vector2f(p.x()+avoid_x,p.y()-avoid_y),
                         Vector2f(p.x()-avoid_x,p.y()+avoid_y));
    }

} // namespace task
