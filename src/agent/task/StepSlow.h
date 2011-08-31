/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef TASK_STEP_SLOW_H
#define TASK_STEP_SLOW_H

#define ENABLE_TASK_STEP_SLOW_LOG

#include "Task.h"
#include "math/Math.hpp"
#ifdef ENABLE_TASK_STEP_SLOW_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task {

    class StepSlow: public Task
    {
    public:
        StepSlow( bool isLeft,
              const math::Vector2f& size, math::AngDeg dir,
              boost::shared_ptr<const StepSlow> preStep,
              float bodyHeight,
              Task* primary );

        virtual bool isTerminable() const;

        virtual bool isDone() const;

        virtual boost::shared_ptr<action::Action> perform();

        /**
         * @return moving which foot
         */
        bool isLeft() const
            {
                return mIsLeft;
            }

        /**
         * @return the size of the step
         */
        const math::Vector2f& size() const
            {
                return mSize;
            }

        const math::Vector2f& sizeAcc() const
            {
                return mSizeAcc;
            }

        /**
         * @return the direction of the step
         */
        math::AngDeg dir() const
            {
                return mDir;
            }

        static void setMaxSizeAcc( const math::Vector2f& v )
            {
                mMaxSizeAcc = v;
            }

        static void setMaxSize( const math::Vector2f& v )
            {
                mMaxSize = v;
            }

        static void setMaxDirAcc(math::AngDeg a)
            {
                mMaxDirAcc = a;
            }

        static void setMaxDir(math::AngDeg a)
            {
                mMaxDir = a;
            }

        static const math::Vector2f& getMaxSizeAcc()
            {
                return mMaxSizeAcc;
            }

        static const math::Vector2f getMaxSize()
            {
                return mMaxSize;
            }

        static math::AngDeg getMaxDirAcc()
            {
                return mMaxDirAcc;
            }

        static math::AngDeg getMaxDir()
            {
                return mMaxDir;
            }

        static math::AngDeg getStepTime()
            {
                return mStepTime;
            }

    protected:
        /// steping left foot or right foot
        bool mIsLeft;

        /// the step size
        math::Vector2f mSize;

        /// the size accerelation
        math::Vector2f mSizeAcc;

        /// the direction change
        math::AngDeg mDir;

        /// configuration of the walking step
        static math::Vector2f mMaxSizeAcc;
        static math::Vector2f mMinSize;
        static math::Vector2f mMaxSize;
        static math::AngDeg mMaxDirAcc;
        static math::AngDeg mMaxDir;
        static math::AngDeg mMinDir;
        static float mStepTime;

    private:
        // this class handls logging
        DECLARE_STATIC_GRAPHIC_LOGGER;
    };

} // namespace task


#endif // TASK_STEP_H
