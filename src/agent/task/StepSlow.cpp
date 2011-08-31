/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "StepSlow.h"
#include "ShiftFoot.h"
#include "SwingFootSlow.h"
#include "LoadFoot.h"

namespace task{

    using namespace std;
    using namespace boost;
    using namespace math;
    using namespace robot::humanoid;
    using namespace action;

    // (1)
    Vector2f StepSlow::mMaxSizeAcc(0.02f, 0.07f);
    Vector2f StepSlow::mMinSize(-0.03, -0.16);
    Vector2f StepSlow::mMaxSize(0.05, 0.05);
    // (2)
    // Vector2f StepSlow::mMinSize(-0.03, -0.12);
    // Vector2f StepSlow::mMaxSize(0.07, 0.12);
    // (3)
//     Vector2f StepSlow::mMaxSizeAcc(0.02f, 0.07f);
//     Vector2f StepSlow::mMinSize(-0.03, -0.14);
//     Vector2f StepSlow::mMaxSize(0.07, 0.14);

    AngDeg StepSlow::mMaxDirAcc = 50.0f;
    AngDeg StepSlow::mMinDir = 0.0f;
    AngDeg StepSlow::mMaxDir = 50.0f;
    float StepSlow::mStepTime = 0.4;

    // #define ENABLE_SPEED_UP

    DEFINE_STATIC_GRAPHIC_LOGGER(StepSlow)

    StepSlow::StepSlow( bool isLeft,
                const Vector2f& size, AngDeg dir,
                shared_ptr<const StepSlow> preStep,
                float bodyHeight,
                Task* primary )
        :Task(-1, primary),
         mIsLeft(isLeft)
    {
        BEGIN_ADD_STATIC_LOG_LAYER(StepSlow)
        ADD_LOG_LAYER("new");
        ADD_LOG_LAYER("speedUp");
        END_ADD_STATIC_LOG_LAYER(StepSlow)

        Vector2f preSize(0,0);
        Vector2f preSizeAcc(0,0);
        AngDeg preDir = 0;
        if ( 0!=preStep.get() ){
            preSize = preStep->size();
            preSizeAcc = preStep->sizeAcc();
            preDir = preStep->dir();
        }

        LOG_PRINTF("new","PreStep: %.3f, %.3f, %.3f ==> %.3f, %.3f, %.3f",
                   preSize.x(), preSize.y(), preDir,
                   size.x(), size.y(), dir);

        mSize = size;
        /////////////////////////////////////////////////////////////
        /// restrict the step size and direction

        //----------------------------------------------------------
        // restrict the turning angle
        AngDeg maxDirAcc = mMaxDirAcc;
        AngDeg maxDir = mIsLeft?mMaxDir:-mMinDir;
        AngDeg minDir = mIsLeft?mMinDir:-mMaxDir;

#ifdef ENABLE_SPEED_UP
        // AngDeg standTurningDist2 = pow2(0.05f);
        // if ( (abs(dir)>mMaxDir && preSize.squareLength() < standTurningDist2&&
        //       size.squareLength() < standTurningDist2 )
        //      || abs(preDir) > mMaxDir ){
        //     // speed up the turnning
        //     maxDir = 8.0f*0.75f;
        //     maxDirAcc = 2.0f*0.75f;
        //     mSize = preSize*0.3f;
        //     LOG_PRINT("speedUp","turnning!");
        // }
#endif

        AngDeg dirAcc = calClipAng( dir, preDir );
        LOG_PRINTF("new","dirAcc: %.3f", dirAcc);
        dirAcc = clamp(dirAcc, -maxDirAcc, maxDirAcc);
        LOG_PRINTF("new","clamped dirAcc: %.3f", dirAcc);
        float maxsizelen = mMaxSize.length();
        float presizelen = preSize.length();
        dirAcc *= max(0.5f, 1-presizelen/maxsizelen );
        LOG_PRINTF("new","restricted by Size dirAcc: %.3f", dirAcc);
        mDir = dirAcc + preDir;
        LOG_PRINTF("new","calculated mDir: %.3f", mDir);

        LOG_PRINTF("new","max dir: %.3f", maxDir);
        mDir = clamp(mDir, minDir, maxDir);
        LOG_PRINTF("new","final mDir: %.3f", mDir);
        Vector2f rotateMov;
        float halfFeetWidth = HUMANOID.getHalfFeetWidth();
        rotateMov.x() = (mIsLeft?1:-1) * sign(mDir)
            * halfFeetWidth * ( 1 - cosDeg(mDir) ) * 2;
        rotateMov.y() = (mIsLeft?-1:1) * halfFeetWidth * sinDeg(mDir) * 2;

        //----------------------------------------------------------
        // restirct the step size

        Vector2f maxSize, minSize;
        maxSize.x() = (mIsLeft?-mMinSize.x():mMaxSize.x());
        maxSize.y() = mMaxSize.y() * cosDeg(mDir);
        minSize.x() = (mIsLeft?-mMaxSize.y():mMinSize.y());
        minSize.y() = mMinSize.y() * cosDeg(mDir);
#ifdef ENABLE_SPEED_UP
        if ( ( abs(mSize.y()) > abs(mMaxSize.y()) && abs(mDir) < 2
               && abs(preDir) < 2 && abs(mSize.x()) < 0.05f )
             /*|| abs(preSize.y()) > mMaxSize.y()*/ ){
            // speed up for/backward
            mSize.x() = preSize.x();
            maxSize.y() = 0.25f;
            minSize.y() = -maxSize.y();
            // but should decrease the ratio of speed
            mSize.x() *= 0.3f;
            mDir = preDir*0.3f;
            LOG_PRINT("speedUp","running!");
        }
        else if ( ( abs(mSize.x())>abs(mMaxSize.x()) && abs(mDir) < 2
                    && abs(preDir) < 2 && abs(mSize.y()) < 0.05f )
                  /*|| abs(preSize.x()) > mMaxSize.x()*/ ){
            // speed up sideward
            maxSize.x() = 0.2f;
            mSize.y() = preSize.y();
            mSize.y() *= 0.3f;
            mDir = preDir*0.3f;
            LOG_PRINT("speedUp","side walking!");
        }
#endif
        if ( mSize.squareLength() > 1 ){
            // to avoid side walk by noise data
            mSize.normalize();
            mSize*=2;
            LOG_PRINTF("new","avoid noise, mSize: %.3f, %.3f",
                       mSize.x(), mSize.y());
        }

        mSize += rotateMov;
        LOG_PRINTF("new","add rotateMov, mSize: %.3f, %.3f",
                       mSize.x(), mSize.y());

        mSizeAcc = mSize - preSize;
        LOG_PRINTF("new","mSizeAcc: %.3f, %.3f", mSizeAcc.x(), mSizeAcc.y());
        mSizeAcc.x() = clamp( mSizeAcc.x(),
                              -mMaxSizeAcc.x(), mMaxSizeAcc.x());
        mSizeAcc.y() = clamp( mSizeAcc.y(),
                              -mMaxSizeAcc.y(), mMaxSizeAcc.y());
        LOG_PRINTF("new","clamped mSizeAcc: %.3f, %.3f",
                   mSizeAcc.x(), mSizeAcc.y());
        mSizeAcc *= cosDeg(mDir);
        LOG_PRINTF("new","restircted by turning mSizeAcc: %.3f, %.3f",
                   mSizeAcc.x(), mSizeAcc.y());
        mSize = preSize + mSizeAcc;

        // restric walk side according to forward or backward
        float maxLen = maxSize.length();
        float len = mSize.length();
        bool slowDown = false;
        if ( len > maxLen ){
            mSize *= (maxLen/len);
        }
        else if( len < 0.03f){
            LOG_PRINTF("new","slow down");
            slowDown = true;
        }

        LOG_PRINTF("new","restricted by X-Y, mSize: %.3f, %.3f",
                   mSize.x(),mSize.y());

        mSize.y() = clamp(mSize.y(), minSize.y(), maxSize.y());
        mSize.x() = clamp(mSize.x(), minSize.x(), maxSize.x());
        LOG_PRINTF("new","final mSize: %.3f, %.3f", mSize.x(), mSize.y());
        mSizeAcc = mSize - preSize;
        LOG_PRINTF("new","final mSizeAcc: %.3f, %.3f",
                   mSizeAcc.x(), mSizeAcc.y());

        //////////////////////////////////////////////////////////
        // create sequences of one step
        const float loft_foot_height = HUMANOID.getMinFootHeight();
        float swingHeight = slowDown?(loft_foot_height):(loft_foot_height*1.8f);

        TransMatrixf t0;
        t0.rotationZ(preDir);
        float feetx = (mIsLeft?1:-1)*HUMANOID.getHalfFeetWidth();
        t0.p().x() = preSize.x();
        t0.p().y() = preSize.y();
        t0.p().x() += feetx;
        t0.transfer(Vector3f(-feetx,0,0));
        TransMatrixf t1;
        t1.identity();
        t1.p().x() = -feetx;
        TransMatrixf t2 = t0;
        t2.inverseTransfer(t1);

        Vector2f startPoint;
        startPoint.x() = t2.p().x() + feetx;
        startPoint.y() = t2.p().y();

        LOG_PRINTF("new","swing start: %.3f, %.3f, %.3f",startPoint.x(),startPoint.y(),-preDir);
        Vector2f swingVec = mSize - startPoint;
        LOG_PRINT_VECTOR3("new",swingVec);
        LOG_PRINTF("new","swingVec.len=%.3f",swingVec.length());
        float totalTime = 0.2;
        float stepTime = 2 * ceil(WM.getAverageStepTime() / serversetting::sim_step) * serversetting::sim_step;
        int n = max(3, int(ceil(totalTime/stepTime)));
        swingVec /= n;
        AngDeg swingAng = (mDir+preDir) / n;
        float a = - (4*swingHeight/n/n);
        float b = 4*swingHeight/n;
        AngDeg rotateFoot = 0;
        if (mSize.y()>0.02){
            if ( stepTime < 0.05f ){
                rotateFoot = 10;
            }
            else{
                rotateFoot = 2.5f;
            }
        }

        for(int i=1; i<n+1; i++){
            float h = a*i*i + b*i;
            shared_ptr<Task> swingFoot
                ( new SwingFootSlow(mIsLeft, startPoint + swingVec*i,
                                h,
                                -preDir + swingAng*i,
                                rotateFoot,
                                bodyHeight, stepTime, this ) );
            mSubTaskList.push_back(swingFoot);
        }

        LOG_FLUSH;
    }

    bool StepSlow::isDone() const
    {
        return Task::isDone();

        if ( !Task::isDone() ){
            return false;
        }

        // see if the leg is on the ground after step
        bool supported = WM.lastPerception().forceResistance().isTouch(
            /*mIsLeft?FRID_LEFT_FOOT:FRID_RIGHT_FOOT TODO*/0);
        return supported;
    }

    shared_ptr<Action> StepSlow::perform()
    {
        // try do sub task
        shared_ptr<Action> act = Task::perform();

        if ( 0==act.get() ){
            // there is no sub task
            shared_ptr<JointAction> jact( new JointAction );
            jact->fill(0);
            act = shared_static_cast<Action>(jact);
        }

        return act;
    }

    bool StepSlow::isTerminable() const
    {
        // if double support, the aciton can be breaked
        return mSubTaskList.size() < 2
            && WM.isTouch(/*FRID_LEFT_FOOT TODO*/0) && WM.isTouch(/*FRID_RIGHT_FOOT*/1);
    }

} // namespace task
