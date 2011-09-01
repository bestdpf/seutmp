/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WorldModel.h 2733 2009-03-31 12:24:31Z cs $
 *
 ****************************************************************************/

#ifndef CORE_WORLD_MODEL
#define CORE_WORLD_MODEL

//#define ENABLE_WORLD_MODEL_LOG

#include "configuration/Configuration.h"
#include "Singleton.hpp"
#include "perception/Perception.h"
#include "perception/Vision.h"
#include "math/TConvexPolygon.hpp"
#include "robot/humanoid/Humanoid.h"
#include "perception/ExtenedKalmanFilter.hpp"
#include "PassModel.h"
#include <set>
#include"Localization.h"
#ifdef ENABLE_WORLD_MODEL_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace core {

    using namespace perception;
    using namespace std;
    using namespace boost;
    using namespace math;
    using namespace serversetting;
    using namespace robot;
    using namespace robot::humanoid;
    using namespace action;
    using namespace perception;

#define MAX_VEL_DIFF 0.15f
#define MAX_POSTION_DIFF 0.03f

    class WorldModel : public Singleton<WorldModel> {
    public:



        /** constructor */
        WorldModel();

        /** Destructor */
        ~WorldModel();

        /**
         * update from the perception and store it
         *
         * @param p the smart pointer to the perception
         *
         * @return successfully updated or not
         */
        bool update(boost::shared_ptr<perception::Perception> p);


        // ===========================
        // Get the golbal information
        // ===========================

        /**
         * @return the global position of the upper torso
         */
        const math::Vector3f& getMyGlobalPos() const {
            return getVisionTrans().pos();
        }

        const math::Vector2f& getMyGlobalPos2D() const {
            return *(const math::Vector2f*)(getVisionTrans().pos().get());
        }

        /**
         * @return the global velocity of the upper torso
         */
        const math::Vector3f& getMyGlobalVel() const {
            return mMyGlobalVel;
        }

        /**
         * @return the local coordination's origin of the agent self
         * in Vector3f formate
         */
        const math::Vector3f& getMyOrigin() const {
            return mMyOriginMatrix.pos();
        }

        /**
         * @return the local coordination's origin of the agent self
         * in Vector2f formate
         */
        const math::Vector2f& getMyOrigin2D() const {
            return *(const math::Vector2f*)(mMyOriginMatrix.pos().get());
        }

        /**
         * @return the local coordination's matrix of the agent self
         */
        const math::TransMatrixf& getMyOriginTrans() const {
            return mMyOriginMatrix;
        }

        math::AngDeg getMyFaceDirection() const {
            return mMyFaceDirection;
        }

        const math::Vector3f& getMyCenterOfMass() const {
            return mMyCenterOfMass;
        }

        const math::Vector3f& getMyGyroRate() const {
            return lastPerception().gyroRate().rate(0);
        }

        math::Vector3f calMyBodyAngRate(math::Vector3f  myRot); //计算mMyBodyAng的变化率， mMyBodyAng即姿态的角度，全局旋转矩阵的欧拉角（YPR顺序，又ZYX）

        math::Vector3f calMyBodyRotatedAng(math::Vector3f  MyBodyAngRate); //计算mMyBodyAng的改变量

        /***********************************end******************************************/
        /////////////////////////add by allen 2010.3.15

        const math::Vector3f& getMyAcc() const {
            return mMyAcc;
        }
        ////////////////////////////////////////////////

        math::AngDeg getMyBodyDirection() const
		{
			if(seenFlagsNum()>=3)
				return mMyBodyDirection;
			else
				return mMyBodyDirWithFlags;
		}

        robot::humanoid::Humanoid::ESkeleton getMySupportFoot() const {
            return mMySupportBone;
        }

        bool isDoubleSupport() {
            return lastPerception().forceResistance().isDoubleSupport();
        }

        /**
         * get our teammates' global position
         *
         * @param i the number of the teammate
         *
         * @return the 3D position of teammate's torso
         */
        const math::Vector3f& getOurGlobalPos(unsigned int i) const {
            std::map<unsigned int, math::Vector3f>::const_iterator iter = mTeammateGlobalPos.find(i);
            if (mTeammateGlobalPos.end() != iter) return iter->second;
            return mIllegalPos;
        }

        const std::map<unsigned int, math::Vector3f>& getOurGlobalPos() const {
            return mTeammateGlobalPos;
        }

        const math::Vector2f& getOurGlobalPos2D(unsigned int i) const {
            return *(const math::Vector2f*)(getOurGlobalPos(i).get());
        }

        /**
         * get opponent's global position
         *
         * @param i the number of the opponent
         *
         * @return the 3D position of opponent's torso
         */
        const math::Vector3f& getOppGlobalPos(unsigned int i) const {
            std::map<unsigned int, math::Vector3f>::const_iterator iter = mOpponentGlobalPos.find(i);
            if (mOpponentGlobalPos.end() != iter) return iter->second;
            return mIllegalPos;
        }

        const std::map<unsigned int, math::Vector3f>& getOppGlobalPos() const {
            return mOpponentGlobalPos;
        }

        /*   const math::Vector2f& getOppGlobalVel(int depth) const				/////terrymimi
                   {
                           return mOpponentGlobalPos2D[0] - mOpponentGlobalPos2D[depth]/0.02/depth;
                   }*/
        const math::Vector2f& getOppGlobalPos2D(unsigned int i) const {
            return *(const math::Vector2f*)(getOppGlobalPos(i).get());
        }
        //////////////////////add by allen 2010.6.7////////////////////

        const int getOurPlayerNumber() {
            return mOurPlayerNum;
        }

        const int getOppPlayerNumber() {
            return mOppPlayerNum;
        }
        ////////////////////////////////////////////////////

        const math::Vector3f& getBallGlobalPos() const
		{
			if(seenFlagsNum()>=3)
				return mBallGlobalPos;
			else
				return Vector3f(mBallGlobalPos2DWithRelInfo.x(), mBallGlobalPos2DWithRelInfo.y(), ball_radius);
		}

        const math::Vector3f& getG2RGlobalPos() const {
            return mG2RGlobalPos;
        } //terry

        const math::Vector3f& getG1RGlobalPos() const {
            return mG1RGlobalPos;
        } //terry

        const math::Vector3f& getBallAveragePos() const {
            return mBallAveragePos;
        }

        const math::Vector2f& getBallGlobalPos2D() const
		{
			if(seenFlagsNum()>=3)
				return *(const math::Vector2f*)(mBallGlobalPos.get());
			else
				return mBallGlobalPos2DWithRelInfo;
		}

        const math::Vector3f& getBallGlobalVel() const {
            return mBallGlobalVel;
        }

        const math::Vector3f& getBallGlobalStopPos() const {
            return mBallGlobalStopPos;
        }

        const math::Vector2f& getBallGlobalStopPos2D() const {
            return *(const math::Vector2f*)(mBallGlobalStopPos.get());
        }

        const math::Vector3f& getInterceptBallGlobalPos() const {
            return mInterceptBallGlobalPos;
        } //camera

        const math::Vector2f& getInterceptBallGlobalPos2D() const //add it
        {
            return *(const math::Vector2f*)(mInterceptBallGlobalPos.get());
        }

        const math::TransMatrixf& getVisionTrans() const {
            return mMyVisionMatrix;
        }

        const math::TransMatrixf& getLeftFootTrans() const;

        const math::TransMatrixf& getRightFootTrans() const;

        const std::map<unsigned int, math::TransMatrixf>& getBoneTrans() const {
            return mBoneTrans;
        }

        // this function name is out date, do not use!!!

        const math::TransMatrixf& getJointTrans(unsigned int id) const {
            return getBoneTrans(id);
        }

        const math::TransMatrixf& getBoneTrans(unsigned int id) const {
            return mBoneTrans.find(id)->second;
        }

        const math::TransMatrixf& getBoneTrans(robot::humanoid::Humanoid::ESkeleton eid) const {
            unsigned int id = HUMANOID.getBoneId(eid);
            return getBoneTrans(id);
        }

        math::AngDeg getJointAngle(unsigned int jid) {
            //const perception::Perception& p=lastPerception();
            //const perception::JointPerception& jp=p.joints();
            return lastPerception().joints().jointAng(jid);
        }

        math::AngDeg getJointRate(unsigned int jid) {
            return lastPerception().joints().jointRate(jid);
        }

        /**
         * get the sum force that applied to the feet
         *
         * @return the vector of force
         */
        const math::Vector3f& getFeetForce() const {
            return mFeetForce;
        }

        /**
         * get the point of sum force that applied to the feet
         *
         * @return the force point
         */
        const math::Vector3f& getFeetForcePoint() const {
            return mFeetForcePoint;
        }

        const math::Vector3f& getMyBodyAng() const {
            return mMyBodyAng;
        }

        // ==========================
        // get simulation information
        // ==========================

        float getSimTime() const {
            return lastPerception().time().now();
        }

        unsigned int getSimCycle() const {
            return mSimCycle;
        }

        float getAverageStepTime() const;

        const perception::Perception& lastPerception() const {
            return *(mPerceptions.back().get());
        }

        const perception::Perception& predictedPerception() const {
            return mPredictedPerception;
        }

        typedef std::deque< boost::shared_ptr<perception::Perception> > TPerceptionDeque;

        const TPerceptionDeque& getPerceptions() const {
            return mPerceptions;
        }

        math::Vector3f getBallPosToBone() const;

        math::Vector3f getPosToBone(math::Vector3f) const;
        // ==========================
        // get game information
        // ==========================

        serversetting::TPlayMode getPlayMode() const {
            return lastPerception().getPlayMode();
        }

        serversetting::TTeamIndex getOurTeamIndex() const {
            return perception::GameState::getOurTeamIndex();
        }

        const std::string& getOurTeamName() const {
            return perception::Vision::ourTeamName();
        }

        unsigned int getMyUnum() const {
            return perception::GameState::unum();
        }

        float getGameTime() const {
            return lastPerception().gameState().gameTime();
        }

        int getOurGoal() const {
            return mOurGoal;
        }

        int getOppGoal() const {
            return mOppGoal;
        }

        /**
         * calculate the closest position to a given position in the array
         * this is useful in calculate who is the closest to a given object
         * @note consider the 2D only!!
         *
         * @param pos the given position
         * @param data the array of candinate position
         * @param min min Id of the array
         * @param max max Id of the array
         * @param minDist the min distance
         *
         * @return the Id of the closest position in array
         */
        unsigned int calClosestIdToPosition(const math::Vector3f& pos,
                const std::map<unsigned int, math::Vector3f>& data,
                float& minDist) const;

        // ==========================
        // prediction
        // ==========================
        /**
         * predict the movement of ball, this function is really simple,
         * just use a model of ball movement
         *
         * @param p the position of ball
         * @param v the velocity of ball
         */
        void predictBall(math::Vector3f& p, math::Vector3f& v) const;

        unsigned int getOppClosestToMe() const {
            return mOppClosestToMe;
        }

        float getOppClosestToMeDistance() const {
            return mOppClosestToMeDist;
        }

        // ==========================
        // high levle judgement
        // ==========================
        bool isGroundBall() const;

        bool isStable() const;

        bool isLeftRolling() const;

        bool isLeftRolled() const;

        bool isRightRolling() const;

        bool isRightRolled() const;

        bool isDiving() const;

        bool isDived() const;

        bool isDived(math::AngDeg angX) const;

        bool isLying() const;

        bool isLied() const;

        bool isLied(math::AngDeg angX) const;

        bool isCloseToGoal() const;

        bool isLeftFall() const;

        bool isLeftFall(math::AngDeg angle) const;

        bool isRightFall() const;

        bool isRightFall(math::AngDeg angle) const;

        bool isFall() const;

        /**
         * check if I am the closest one to the ball
         */

        int GetOppCount(Vector2f);

        bool amIFastestToBallOfOurTeam() const;

        float howIFasterToBallThanOpps() const;

        float getMyInterceptBallTime() const {
            return mMyInterceptBallTime;
        }

        bool isTouch(unsigned int id) const {
            return lastPerception().forceResistance().isTouch(id);
        }

        /**
         * This function tells whether the specified foot is contact with the ball.
         */
        bool isTouchBall(unsigned int id) const;

        /**
         * @return the position of opponent who is closest in their team
         */
        const math::Vector3f& getOppFastestToBallPos() const {
            return getOppGlobalPos(mOppFastestToBall);
        }

        float getOppFastestToBallDistance() const {
            return mOppFastestToBallDist;
        }

        float getOurFastestToBallDistance() const {
            return mOurFastestToBallDist;
        }

        const math::Vector3f& getOurFastestToBallPos() const {
            return getOurGlobalPos(mOurFastestToBall);
        }

        unsigned int getOurFastestToBallNum() const {
            return mOurFastestToBall;
        }

        unsigned int getHearOurFastestToBallNum() const {
            return mHearOurFastestToBall;
        }

        void setHearOurFastestToBallNum(unsigned int num) {
            mHearOurFastestToBall = num;
        }

        bool isPlayerInField(const math::Vector2f& pos, float margin = 0);

        float getLostSimTime() const {
            return mLostSimTime;
        }

        bool isHaveOppInArea(math::TConvexPolygon<float> area);

        void setDribbleSide(bool isLeft) /////terrymimi
        {
            isLeftDribble = isLeft;
        }

        bool getDribbleSide() const {
            return isLeftDribble;
        }

        bool isBallToOurGoal(void);

        bool isGameStateChanged() const {
            if (mPerceptions.size() > 1)
                return lastPerception().gameState().getPlayMode() != mPerceptions[mPerceptions.size() - 2]->gameState().getPlayMode();
            else
                return false;
        }

        void setCameraRot(math::Vector3f& rot) //vision-me
        {
            cameraRot = rot;
        }

        int getFlagNumbersISee()const //vision-me
        {
                const Vision* v = lastPerception().vision().get();
                if (v != NULL) {
                        set<Vision::FID> flags = (*v).getStaticFlagSet();
                        int num = flags.size();

                        return num;
                }

                return 0;
        }

        /*bool canISeeTheBall()const //vision-me
        {
                //shared_ptr<const Vision> vp = lastPerception().vision();
                const Vision* v = lastPerception().vision().get();
                if (v != NULL) {
                        return v->canISeeBall();
                }
                return false;
        }*/

        /*bool canISeeTheBallYu()//YuRobo 在得不到视觉的两个周期内，返回上次的值
        {
                //shared_ptr<const Vision> vp = lastPerception().vision();
                const Vision* v = lastPerception().vision().get();
                if (v != NULL) {
                        lastCanISeeBall = v->canISeeBall();
                        return v->canISeeBall();
                }
                return lastCanISeeBall;
        }*/

        /*bool canISee(perception::Vision::FID fid)const //YuRobo
        {
                //shared_ptr<const Vision> vp = lastPerception().vision();
                const Vision* v = lastPerception().vision().get();
                if (v != NULL) {
                        return v->canISee(fid);
                }
                return false;
        }*/

        const math::Vector3f& getBallLaPol()const //jia
        {
            return ballLaPol; //得到球的视觉极坐标
        }

        void setSearchSpeedX(float speed) /////terry
        {
            mSearchSpeed.x() = speed;
        }

        void setSearchSpeedY(float speed) /////terry
        {
            mSearchSpeed.y() = speed;
        }

        const math::Vector2f& getSearchSpeed() const /////terry
        {
            return mSearchSpeed;
        }

        math::Vector3f genWalkSizeLackFlag() const; //terry

        math::Vector3f genWalkDirLackFlag() const; //terry

        /*boost::shared_ptr<const perception::Vision> getLatestVision() const //terry
        {
                return mLatestV;
        }*/

        float getLatestFlagNumISee() //terry
        {
            const Vision* v = lastPerception().vision().get();
            if (v != NULL) {
                set<Vision::FID> flags = (*v).getStaticFlagSet();
                latestFlagNum = flags.size();
            }
            return latestFlagNum;
        }


        //--------------------------------------------------TT

        bool isItInOurPenaltyZone(Vector2f& a) const {
            if (a[0]>-half_field_length && a[0]<-half_field_length + penalty_length
                    && abs(a[1]) < half_penalty_width)
                return true;
            else
                return false;
        }

        float timeAfterLastSeeEnoughFlags() const {
            return getSimTime() - mLastTimeSeeEnoughFlags;
        }

        	int FindOneInFrontMe()
        {
           Vector2f myPos=getMyGlobalPos2D();
            //int max=-1;
            Vector2f temp;
            for(int i=1;i<10;i++)
            {
                temp=getOurGlobalPos2D(i);
                if(temp.x()>half_field_length||temp.x()<-half_field_length||temp.y()>half_field_width||temp.y()<half_field_width)
                    continue;
                if(temp.x()>myPos.x())
                {
                    return i;
                }
            }
            return -1;
        }

        void setLocalWalkSign(bool b) {
            localWalkSign = b;
        }

        bool getLocalWalkSign() const {
            return localWalkSign;
        } //used only in Walk

        void setRushToGoalSign(bool b) {
            rushToGoalSign = b;
        }

        bool getRushToGoalSign() const {
            return rushToGoalSign;
        } //used only in Walk

        void setIsLeftFootKick(bool b) {
            isLeftFootKick = b;
        }//////////////////////////////

        bool getIsLeftFootKick() const {
            return isLeftFootKick;
        }///////////////////////////////
        //===================================================================

        void setNowFormation(configuration::Formation::FormationType p_ft) {
            nowFT = p_ft;
        }

        configuration::Formation::FormationType getNowFormation() {
            return nowFT;
        }

        void setBestZone(int p_X, int p_Y) {
            bestZoneX = p_X;
            bestZoneY = p_Y;
        }

        int getBestZoneX() {
            return bestZoneX;
        }

        int getBestZoneY() {
            return bestZoneY;
        }

        bool getIsAttacking() {
            return isAttacking;
        }

        void setIsAttacking(bool att) {
            isAttacking = att;
        }

        //=================================================================TT REL

        struct ObjectRelInfo {
            bool canSee;
            Vector2f relPos2D; //+x: right, +y: forward
            Vector2f pol2D; //(dist,ang); ang:left+, right-
        };
	/**	this function is used to set data to a new ObjectRelInfo
	*	it is to overwrite the {} method of struct's build function
	*	because this method is not supported on some OSs like FreeBSD
	*/
	ObjectRelInfo	newObjectRelInfo(bool a,Vector2f b,Vector2f c){
		ObjectRelInfo ret;
		ret.canSee=a;
		ret.relPos2D=b;
		ret.pol2D=c;
		return ret;
	}
        struct BlockInfo {
            float dist;
            float angC;
            float angL; //angC+theta
            float angR; //angC-theta
            //bool operator < (const BlockInfo& a);
        };

        /*bool BlockInfo::operator < (const BlockInfo& a){
                return dist<a.dist;
        }*/

        bool canSeeBall() {
            return mBallRelInfo.canSee;
        }

        const Vector2f& getBallRelPos2D() {
            return mBallRelInfo.relPos2D;
        }

        const Vector2f& getBallPol2D() {
            return mBallRelInfo.pol2D;
        }

        const math::Vector2f& getBallRelVel2D() {
            return mBallRelVel2D;
        }

        bool canSeeFlag(Vision::FID fid) {
            return mFlagRelInfoMap[fid].canSee;
        }

        const Vector2f& getFlagRelPos2D(Vision::FID fid) {
            return mFlagRelInfoMap[fid].relPos2D;
        }

        const Vector2f& getFlagPol2D(Vision::FID fid) {
            return mFlagRelInfoMap[fid].pol2D;
        }

        /*const map<Vision::FID,ObjectRelInfo>& getFlagRelInfoMap() const {
                return mFlagRelInfoMap;
        }*/

        const BlockInfo& getBallBlock() const {
            return mBallBlock;
        }

        const std::list<BlockInfo>& getBlockList() const {
            return mBlockList;
        }

        //you should use this function only when you see only 2 flags
        //TT, March, MMXI
        Vector2f calMyGlobalPos2DWithTwoFlags();

		//TT, July, MMXI
		void calMyBodyDirWithFlags();

		//TT, July, MMXI
		void calBallGlobalPos2DWithRelInfo();

        //TT, April, MMXI
        //different from old function, this will return an old num if there was no vision

        int seenFlagsNum() const {
            int num = 0;

            FOR_EACH(iter, mFlagRelInfoMap) {
                if (true == iter->second.canSee)
                    num++;
            }
            return num;
        }

        int seenRightFlagsNum() const {
            int num = 0;

            FOR_EACH(iter, mFlagRelInfoMap) {
                if (true == iter->second.canSee) {
                    if (Vision::F1R == iter->first ||
                            Vision::F2R == iter->first ||
                            Vision::G1R == iter->first ||
                            Vision::G2R == iter->first) {
                        num++;
                    }
                }
            }
            return num;
        }


        Vector2f calObjRelPos2D(const Vector3f& objPolToVisionSensor);

        math::Vector2f transRelPosToGlobalPos(const Vector2f& originGlobalPos, const Vector2f& relPos) {
            AngDeg theta = getMyBodyDirection() - 90.0f;
            return originGlobalPos + Vector2f(relPos.x() * cosDeg(theta) - relPos.y() * sinDeg(theta),
                    relPos.x() * sinDeg(theta) + relPos.y() * cosDeg(theta));
        }

        math::Vector2f transGlobalPosToRelPos(const Vector2f& originPos, const Vector2f& destPos) {
            AngDeg theta = 90.0f - getMyBodyDirection();
            Vector2f deltaV2f = destPos - originPos;
            return Vector2f(deltaV2f.x() * cosDeg(theta) - deltaV2f.y() * sinDeg(theta),
                    deltaV2f.x() * sinDeg(theta) + deltaV2f.y() * cosDeg(theta));
        }
        //TT for test
        void logPrintSeenFlags();

    private:
        // =====================
        //  private functions
        // =====================

        // ============================
        //  world model update functions
        // ============================
        /** Update the information from server */
        bool updatePerception(boost::shared_ptr<perception::Perception> newP);

        void localization();

        math::Matrix3x3f calcVisionSensorRotation(const math::Vector3f& myPos,
                const Vision& v,
                Vision::FID flagA,
                Vision::FID flagB,
                Vision::FID flagC) const;

        /**
         * calculate the global position of vision sensor
         *
         * @param v the vision information
         *
         * @return the global position of vision sensor
         */
        math::Vector3f calcVisionSensorPosition(const Vision& v);

        math::Vector3f calcVisionSensorPositionWith2Flags(const Vision& v);

        math::Vector3f calcVisionPositionWithThreeFlags(const math::Vector3f& posA, const math::Vector3f& polA, const math::Vector3f& lposA,
                const math::Vector3f& posB, const math::Vector3f& polB, const math::Vector3f& lposB,
                const math::Vector3f& posC, const math::Vector3f& polC, const math::Vector3f& lposC) const;

        math::Vector3f calcVisionPositionWithThreeFlags(const Vision& v, Vision::FID fA, Vision::FID fB, Vision::FID fC) const;

        math::Vector3f calcVisionPositionWithTwoFlags(const math::Vector3f& posA, const math::Vector3f& polA,
                const math::Vector3f& posB, const math::Vector3f& polB) const;

        math::Vector3f calcVisionPositionWithTwoFlags(const Vision& v, Vision::FID fA, Vision::FID fB) const;

        void updateBall();

        void updateSelf();

        void updatePlayers();

        /**
         * predicted the real intercepted position and who is the fastest
         * to the ball
         *
         * @param data the position of player
         * @param speed the speed of player
         * @param minTime the time of fastest player to intercepted the ball
         *
         * @return the number of fastest player
         */
        unsigned int calFastestIdToBall(const std::map<unsigned int, math::Vector3f>& data,
                float speed, float& minTime);

        float predictInterceptBall(math::Vector3f& posBall, math::Vector3f& velBall,
                const math::Vector3f& pos, float speed, float maxTime) const;


        //TT, MMXI
        //for relative information
        void updateObjectRelInfo();

        Vector2f calObjPol2D(const Vector2f& relPos2D);
        void buildBlocks();


    private:
        unsigned int mSimCycle;

        float mStartSimTime;

        float mLostSimTime;

        float mLastTimeSeeEnoughFlags;

        /** the deque of perception from the server */
        TPerceptionDeque mPerceptions;
        const static unsigned int max_perception_size;

        std::map<unsigned int, math::TransMatrixf> mBoneTrans;

        /////////// Illegal Position ///////
        const static math::Vector3f mIllegalPos;

        /////////// Ball ///////////////
        math::Vector3f mBallGlobalPos;
		math::Vector2f mBallGlobalPos2DWithRelInfo; //TT
        math::Vector3f mG2RGlobalPos; //terry
        math::Vector3f mG1RGlobalPos; //terry
        math::Vector3f mBallGlobalVel;
        math::Vector3f mBallAveragePos;
        math::TVector<PVExtenedKalmanFilter<float>, 3 > mBallPvekf;
        math::Vector3f mBallGlobalStopPos;
        math::Vector3f mInterceptBallGlobalPos;
        float mMyInterceptBallTime;

        ////////// Player /////////////
        std::map<unsigned int, math::Vector3f> mTeammateGlobalPos;
        std::map<unsigned int, math::Vector3f> mOpponentGlobalPos;
        int mOurPlayerNum; //our team players
        int mOppPlayerNum; //opp team players
        unsigned int mOppClosestToMe;
        float mOppClosestToMeDist;

        /// the matrix of the vision sensor
        math::TransMatrixf mMyVisionMatrix;

        math::Vector3f mMyGlobalVel;
        math::AngDeg mMyFaceDirection;
        math::Vector3f mMyCenterOfMass;
        math::Vector3f mMyBodyAng;
        math::Vector3f mMyLastBodyAng; //dubai
        math::Vector3f cameraRot; //vision-me
        math::Vector3f mMyGlobalPos; //vision-me
        math::Vector3f ballLaPol; //jia

        /// the coordination for self motion
        math::TransMatrixf mMyOriginMatrix;
        math::AngDeg mMyBodyDirection;
		math::AngDeg mMyBodyDirWithFlags; //TT

        /// the global position of the force center in feet
        math::Vector3f mLeftFootForceCenter;
        math::Vector3f mRightFootForceCenter;
        /// the force of feet
        math::Vector3f mFeetForcePoint;
        math::Vector3f mFeetForce;

        robot::humanoid::Humanoid::ESkeleton mMySupportBone;

        /// the predicted perception
        perception::Perception mPredictedPerception;

        ///////////////////// for soccer game ///////////////////
        unsigned int mOurFastestToBall;
        unsigned int mHearOurFastestToBall;
        float mOurFastestToBallDist;
        float mOurFastestToBallTime;
        unsigned int mOppFastestToBall;
        float mOppFastestToBallDist;
        float mOppFastestToBallTime;

        int mOurGoal;
        int mOppGoal;

        bool isLeftDribble;

        math::Vector2f mSearchSpeed; //terry

        boost::shared_ptr<const perception::Vision> mLatestV; //terry

        math::Vector3f preBallPol; //terry

        math::Vector3f preG2RPol; //terry

        float latestFlagNum; //terry

        //-------------------------------------------TT xxxxxxxxxxxxxxxxxxxxxxxxxx
        bool localWalkSign;
        bool rushToGoalSign;
        bool isLeftFootKick;
        //----------------------------------------------

        //==================================================TT REL
        ObjectRelInfo mBallRelInfo; //for ball
        std::map<Vision::FID, ObjectRelInfo> mFlagRelInfoMap; //for 8 flags

        BlockInfo mBallBlock; //for ball, but we dont know whether he see ball or not from here
        std::list<BlockInfo> mBlockList; //for players' body

        math::Vector2f mBallRelVel2D;

        //===========================================================
        math::Vector3f mMyAcc;

        //now formationtype add by allen
        configuration::Formation::FormationType nowFT;
        int bestZoneX;
        int bestZoneY;
        bool isAttacking;
        /////////////////////////
        DECLARE_GRAPHIC_LOGGER;

    }; //end of class WorldModel

#define WM core::WorldModel::GetSingleton()

} //end of namespace core

#endif // CORE_WORLD_MODEL

