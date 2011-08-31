/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/


#ifndef TASK_WALK_SLOW_H
#define TASK_WALK_SLOW_H

#define ENABLE_TASK_WALK_SLOW_LOG

#include "Task.h"
#include "math/Math.hpp"
#ifdef ENABLE_TASK_WALK_SLOW_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif


namespace task{

    class WalkSlow: public Task
    {
    public:
        /**
         * create a walk task by walking target and
         * the direction while reach the target position
         *
         * @param target target position
         * @param direction the direction while reach the target position
         * @param primary the primary task which create this task
         *
         */
        WalkSlow( const math::Vector2f& target,
              math::AngDeg direction,
              bool avoidBall,
              Task* primary = NULL );

        virtual bool isDone() const;

        virtual bool revise( boost::shared_ptr<const Task> rt );

        virtual void updateSubTaskList();

        static void setWalkHeight( float h )
            {
                mWalkHeight = h;
            }

        static float getWalkHeight()
            {
                return mWalkHeight;
            }

    protected:
        /// class for path palnning
        class State
        {
            math::Vector2f mP;
            math::Vector2f mPreP;
            float mCosted;
            float mMoreCost;
        public:
            State(const math::Vector2f& p, float costed, float moreCost)
                :mP(p), mCosted(costed), mMoreCost(moreCost)
                {
                }

            const math::Vector2f& p() const
                {
                    return mP;
                }

            float moreCost() const
                {
                    return mMoreCost;
                }

            float cost() const
                {
                    return mCosted + mMoreCost;
                }

            float costed() const
                {
                    return mCosted;
                }

            State previous() const
                {
                    return State(mPreP,0,0);
                }

            void setPrevious(const State& p)
                {
                    mPreP = p.p();
                }

            bool sameAs(const State& r) const
                {
                    return fabs(mP.x()-r.p().x()) < 0.01f
                        && fabs(mP.y()-r.p().y()) < 0.01f;
                }

            bool operator<(const State& r) const
                {
                    return cost() < r.cost();
                }

        };

        /**
         * plan the templated walking target position,
         * the robot should avoid to collide with ball, wall and others
         */
        void planPath();

        /**
         * generate the possible state to go
         *
         * @param start the start state
         * @param target the target state
         * @param res return the possible state from start state
         */
        void generatePossibleState( const State& start, const State& target,
                                    const State& final,
                                    std::vector<State>& res,
                                    std::vector<State>& unReach) const;

        /**
         * this function setup the blocks in the field,
         * the robot should avoid to collide these blocks while walking,
         * currently, the blocks are segments
         */
        void setupBlocks();

        /**
         * build a block according to the relative position with the robot
         * and the given radius
         *
         * @param p the postion of the block
         * @param radius the radius of the block
         *
         * @return the block Segment2f
         */
        math::Segment2f buildBlock(const math::Vector2f& p, float radius) const;

    private:
        /// the target of the walk task
        math::Vector2f mTarget;

        /// the desired driection while reach the target
        math::AngDeg mDirection;

        math::Vector2f mPreSize;

        /// vaiables of path planning
        math::Vector2f mPlanTarget;
        math::AngDeg mPlanDir;

        /// blocks to avoid collding
        std::vector<math::Segment2f> mBlocks;

        /// the error threshold for isDone
        math::Vector2f mSizeErrorThreshold;
        math::AngDeg mDirErrorThreshold;

        static float mWalkHeight;

        float mIsWalking;

        /// should I avoid the ball
        bool mAvoidBall;

        // this class handls logging
        DECLARE_STATIC_GRAPHIC_LOGGER;
    };


} // namespace task

#endif // TASK_WALK_H
