/* 
 * File:   PassModel.h
 * Author: robocup
 *
 * Created on 2011年6月11日, 下午4:21
 */

#ifndef PASSMODEL_H
#define	PASSMODEL_H
#include "core/WorldModel.h"
#include "core/SayAndHearModel.h"
#include"math/Math.hpp"
namespace core {
#define HISTORY 10

    class VisionRobot {
        friend class PassModel;
    public:

        VisionRobot() :
        mLastSeeTime(-1), mHistoryFull(false), mGlobalOffset(0), mHaveSeen(false), mUnum(100), mIsOur(false),mHaveHeard(false)
        {
            for (int i = perception::Vision::TORSO; i < perception::Vision::PID_NULL; i++) {
                mIsSeen[i] = false;
            }
        };
        //与全局坐标系X轴夹角
        //   math::AngDeg calcGlobalDirection();
        //   bool calcDirection();
        void calcInfo(); // update if seen
        const static math::Vector3f mErrPol;

        bool getIsFallen() {
            return mIsFallen;
        }

        bool getHaveSeen() {
            return mHaveSeen;
        }
        //每次视觉周期更新
        bool calcGlobalVel();
       //update every vision

          math::Vector2f getMGlobalPosAll2D() const {
            return mGlobalPosAll;
        }
          math::Vector3f getRelPos() const {
              return mRelPos;
          }
    private:

        math::Vector3f rotationPoints(const math::Vector3f& p);
        bool calcRotationMat(math::AngDeg roll, math::AngDeg pitch, math::AngDeg yaw);
        bool calcHeight();
        //不一定准确
        void calcIsFacedToMe();
        void calcIsFacedToBall();
        void calcIsWalkedToBall();
        ///////////////////////////////////
        void calcWalkDirection();
        void calcPos();
        void calcTimeToBall();
        math::Matrix3x3f mRotationMat;
        math::Vector3f mGlobalPos;
        math::Vector3f mHistoryGlobalPos[HISTORY];
        bool mHistoryFull;
        bool mHaveSeen;
        int mGlobalOffset;
        math::AngDeg mGlobalWalkDirection;



        math::Vector3f mRelPos;
        math::Vector3f mPolPos;
        math::Vector3f mGlobalVel;
        float mHeight;
        bool mIsFallen;

        bool mIsOur;
        bool mIsFacedToMe;
        bool mIsFacedToBall;
        bool mIsWalkedToBall;

        unsigned int mUnum;

        float mTimeToBall;
        math::Vector3f mBodyPartPol[perception::Vision::PID_NULL];
        math::Vector3f mBodyPartRel[perception::Vision::PID_NULL];
        math::Vector3f mBodyPartGlobal[perception::Vision::PID_NULL];
        math::Vector3f mBodyPartVisionRel[perception::Vision::PID_NULL];
        bool mIsSeen[perception::Vision::PID_NULL];
        float mLastSeeTime;
        //hear
        math::Vector2f mHearGlobalPos;
        math::Vector2f mHearBallPos;
        math::AngDeg mHearBodyDirection;
        float mlastHearTime;
        bool mHaveHeard;
        bool mHearFallen;
        //both
        math::Vector2f mGlobalPosAll;

    };

    class PassModel : public Singleton<PassModel> {
        
    public:
        PassModel();
        //    PassModel(const PassModel& orig);
        ~PassModel();
        void update();
        void updateByListen();
typedef std::map<unsigned int, VisionRobot> TSeenPlayer;
        TSeenPlayer& getOurTeammates() {
            return mOurTeammates;
        }

        TSeenPlayer& getOppTeammates() {
            return mOppTeammates;
        }

        unsigned int getOurFastestID() {
            return mOurFastestID;
        }

        float getOurMinTimeToBall() {
            return mOurMinTimeToBall;
        }

        unsigned int getOppFastestID() {
            return mOppFastestID;
        }

        float getOppMinTimeToBall() {
            return mOppMinTimeToBall;
        }

        bool choosePass();
bool canPass();
        unsigned int getBestPassID() const {
            return mBestPassID;
        }
        bool isCanPass() const {
            return mCanPass;
        }

    private:
        void updateTeammates(const perception::Vision::TTeamPolMap& ourPol, TSeenPlayer& teammates, bool isOur);
        void calcOurFastestToBall();
        void calcOppFastestToBall();
        
        
        bool  isBlocked(unsigned int ourTeammateID);


        std::set<unsigned int> mPassSet;
        TSeenPlayer mOurTeammates;
        TSeenPlayer mOppTeammates;

        unsigned int mOurFastestID;
        float mOurMinTimeToBall;
        unsigned int mOppFastestID;
        float mOppMinTimeToBall;

        bool mCanPass;
        float mCanPassTime;
        unsigned int mBestPassID;

    };
}

#define PM core::PassModel::GetSingleton()

#endif	/* PASSMODEL_H */

