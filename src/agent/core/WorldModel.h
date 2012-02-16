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

#include "configuration/Configuration.h"
#include "Singleton.hpp"
#include "perception/Perception.h"
#include "perception/Vision.h"
#include "math/TConvexPolygon.hpp"
#include "robot/humanoid/Humanoid.h"
#include "perception/ExtenedKalmanFilter.hpp"
#include "PassModel.h"
#include <set>

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
         * @return the global position of the eye (camera)
         */
        const math::Vector3f& getMyGlobalPos() const {
          return getMyTorsoGlobalPos();  
	  //return getVisionTrans().pos();
        }
	/**added by dpf
	 * get my torso global pos, not the eye (ie camera)
	 * */
	const math::Vector3f& getMyTorsoGlobalPos() const{
	    return getBoneTrans(robot::humanoid::Humanoid::HEAD).pos();
	}
	
	const math::Vector2f& getMyTorsoGlobalPos2D() const{
	    return *(const math::Vector2f*)(getBoneTrans(robot::humanoid::Humanoid::HEAD).pos().get());
	}
	
        const math::Vector2f& getMyGlobalPos2D() const {
          return getMyTorsoGlobalPos2D();  
	  //return *(const math::Vector2f*)(getVisionTrans().pos().get());
        }

	//dpf test
	//attention, its p() is not glboal pos, but matrix is global matrix
	//use getBoneTrans(TORSO) as soon as possible;
	const math::TransMatrixf getDpfBodyTransMatrix() const {
	  return mDpfBodyTransMatrix;
	}
	
	//dpf test
	const math::TransMatrixf updateDpfBodyTransMatrix() const;
        /**
         * @return the global velocity of the upper torso
         */
	///dpf test, calculate the mVisionBodyAngZ, if no flags can be seen, return false;
	///attention this method is not very good to use, espcially when the body is not stand up straitly.
	bool calcVisionBodyAngZ(const Vision &p);
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
        /**
	 * by dpf
	 * get my Rel com, not lean rel!!!
	 */
	const math::Vector3f& getMyRelCenterOfMass() const {
            return mMyRelCenterOfMass;
        }
        
        const math::Vector3f& getMyGyroRate() const {
            return lastPerception().gyroRate().rate(0);
        }
        
        /***********************************end******************************************/
        /////////////////////////add by allen 2010.3.15

        const math::Vector3f& getMyAcc() const {
            return mMyAcc;
        }
          //test by dpf reset the global pos from acc-sensor to pos given
        void accGlobalPosRest(const math::Vector3f pos){
	     mMyAccPosGlobal=pos;
	}
        //test by dpf reset the global vec from acc-sensor to zero;
        void accVecGlobalReset(){
	     mMyAccVelGlobal.zero();
	}
        ////////////////////////////////////////////////

        math::AngDeg getMyBodyDirection() const {
	   return mMyBodyDirection;
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

        const math::Vector3f& getBallGlobalPos() const {
	  /*
            if (seenFlagsNum() >= 3)
                return mBallGlobalPos;
            else {
                return mBallGlobalPos3DWithRelInfo;
            }
            */
	  //dpf comment the above old lines, i think it is useless and may make mistakes.
	  return mBallGlobalPos;
        }
        const math::Vector3f& getBallAveragePos() const {
            return mBallAveragePos;
        }

        const math::Vector2f& getBallGlobalPos2D() const {
	  /*
            if (seenFlagsNum() >= 3)
                return *(const math::Vector2f*)(mBallGlobalPos.get());
            else
                return mBallGlobalPos2DWithRelInfo;
	    */
	  return *(const math::Vector2f*)(mBallGlobalPos.get());
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
	  //test by dpf
	  return mDpfMyVisionMatrix;
          // return mMyVisionMatrix;
        }
	//dpf test
	void bodyReset() {
	  //rotationZ -90 degrees because the x-y plane not same between two coordination systems;
	  mDpfBodyTransMatrix.rotationZ(-90);;
	  return ;
	}
	//dpf test, reset to bodyAng Vector3f bodyAng(xAng,yAng,zAng),order Y-X-Z, in degrees
	int bodyReset(math::Vector3f bodyAng){
	  mDpfBodyTransMatrix.rotationZ(-90);
	  mDpfBodyTransMatrix.rotateLocalY(bodyAng.y());
	  mDpfBodyTransMatrix.rotateLocalX(bodyAng.x());
	  mDpfBodyTransMatrix.rotateLocalZ(bodyAng.z()+90);
	  return 0; 
	}
        const math::TransMatrixf& getLeftFootTrans() const;

        const math::TransMatrixf& getRightFootTrans() const;

        const std::map<unsigned int, math::TransMatrixf>& getBoneTrans() const {
            return mBoneTrans;
        }
	//added by dpf, lean rel trans
	const std::map<unsigned int, math::TransMatrixf>& getBoneLocalTrans() const {
            return mBoneLocalTrans;
        }
        //rel trans, not lean rel
        const std::map<unsigned int, math::TransMatrixf>& getBoneRelTrans() const {
            return mBoneRelTrans;
        }
        // this function name is out date, do not use!!!

        const math::TransMatrixf& getJointTrans(unsigned int id) const {
            return getBoneTrans(id);
        }

        const math::TransMatrixf& getBoneTrans(unsigned int id) const {
            return mBoneTrans.find(id)->second;
        }
	//added by dpf, lean rel trans
	const math::TransMatrixf& getBoneLocalTrans(unsigned int id) const {
            return mBoneLocalTrans.find(id)->second;
        }
        //added by dpf, rel trans, not lean rel
        //added by dpf
	const math::TransMatrixf& getBoneRelTrans(unsigned int id) const {
            return mBoneRelTrans.find(id)->second;
        }
        const math::TransMatrixf& getBoneTrans(robot::humanoid::Humanoid::ESkeleton eid) const {
            unsigned int id = HUMANOID.getBoneId(eid);
            return getBoneTrans(id);
        }
	//added by dpf, lean rel trans
	const math::TransMatrixf& getBoneLocalTrans(robot::humanoid::Humanoid::ESkeleton eid) const {
            unsigned int id = HUMANOID.getBoneId(eid);
            return getBoneLocalTrans(id);
        }
        //rel trans, not lean rel
        const math::TransMatrixf& getBoneRelTrans(robot::humanoid::Humanoid::ESkeleton eid) const {
            unsigned int id = HUMANOID.getBoneId(eid);
            return getBoneRelTrans(id);
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
	/**
	 * comment by dpf
	 * x() is forward-backward lean angle, when we stand it is 0
	 * y() is left-right angle
	 * z() is bodyDirection, but when we face to opp goal, it is 90, not zero
	 */
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
//dpf comment here, this function return
//        math::Vector3f getBallPosToBone() const;
//global pos to body rel pos, but it maybe useless, take care before you use it 
//because you are more often use rel pos to cal global pos
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
        // high levle judgement, don't use them (comment by dpf)
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

        const math::Vector3f& getBallLaPol()const //jia
        {
            return ballLaPol; //get ball's pol pos.
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

        float getLatestFlagNumISee() //terry
        {
            const Vision* v = lastPerception().vision().get();
            if (v != NULL) {
                set<Vision::FID> flags = (*v).getStaticFlagSet();
                latestFlagNum = flags.size();
            }
            return latestFlagNum;
        }

        float timeAfterLastSeeEnoughFlags() const {
            return getSimTime() - mLastTimeSeeEnoughFlags;
        }

        void setNowFormation(configuration::Formation::FormationType p_ft) {
            nowFT = p_ft;
        }

        configuration::Formation::FormationType getNowFormation() {
            return nowFT;
        }
/** comment by dpf, i don't know what's the below function
 * don't use them before ask allen, maybe allen's work.
 * i think it needs deleted!
 * */
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
        ObjectRelInfo newObjectRelInfo(bool a, Vector2f b, Vector2f c) {
            ObjectRelInfo ret;
            ret.canSee = a;
            ret.relPos2D = b;
            ret.pol2D = c;
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

       // Vector2f calMyGlobalPos2DWithTwoFlags();
	//dpf added, one could in fact get the pos when only 1 flags is seen
	//Vector3f calDpfGlobalPosWithOneFlags(const Vision& v);
	//dpf added, use new method to calc vision global pos
	Vector3f calDpfVisionGlobalPos(const Vision&v);
	//dpf added, one could get the pos when only ball is seen, we guess ball is not moved
	Vector3f calDpfGlobalPosWithOnlyBall();
        int seenFlagsNum() const {
            int num = 0;

            FOR_EACH(iter, mFlagRelInfoMap) {
                if (true == iter->second.canSee)
                    num++;
            }
            return num;
        }

        Vector2f calObjRelPos2D(const Vector3f& objPolToVisionSensor);
	Vector3f calObjRelPos(const Vector3f& objPolToVisionSensor);
	/**
	  * calculate originGlobalPos's rel pos relPos 's globalPos;
	  * used in goTo before kick, by TT
	   */
        math::Vector2f transRelPosToGlobalPos(const Vector2f& originGlobalPos, const Vector2f& relPos) {
	    AngDeg theta = getMyBodyDirection() - 90.0f;
            return originGlobalPos + Vector2f(relPos.x() * cosDeg(theta) - relPos.y() * sinDeg(theta),
                    relPos.x() * sinDeg(theta) + relPos.y() * cosDeg(theta));
	  
	}

        math::Vector2f transGlobalPosToRelPos(const Vector2f& originPos, const Vector2f& destPos) {
	  AngDeg theta =90.0f-getMyBodyDirection();
            Vector2f deltaV2f = destPos - originPos;
            return Vector2f(deltaV2f.x() * cosDeg(theta) - deltaV2f.y() * sinDeg(theta),
                    deltaV2f.x() * sinDeg(theta) + deltaV2f.y() * cosDeg(theta));
        }
        /**
	 * dpf rewrite
	 */
        math::Vector2f transRelPosToGlobalPos(const Vector2f& relPos){
	  return *(math::Vector2f*)(transRelPosToGlobalPos(Vector3f(relPos.x(),relPos.y(),0.0f))).get();
	}
	/**
	 * dpf rewrite
	 */
	math::Vector2f transGlobalPosToRelPos(const Vector2f& globalPos){
	  return *(math::Vector2f*)(transGlobalPosToRelPos(Vector3f(globalPos.x(),globalPos.y(),0.0f))).get();
	}
        /**dpf rewrite it using transfer matrix
          *from the global pos to the eye-rel pos, ie position relative to the camera
          *there are three types of position
          *first, global position with the center of the playground is zero point
          *second, camera rel pos, position relative to the camera
          *third, body rel pos, which is often called rel pos, with is relative to the \
          *body,
	  *there are two kinds of body rel pos, 
	  *1. only consider body's rotationZ
	  *2. also consider other's rotation
	  *below two funcitons is about 2, but it works more near to 1, maybe it is more stable.
	  **/
	math::Vector3f transGlobalPosToRelPos(const Vector3f &destPos){
	  math::TransMatrixf relToGlobalTrans;
	  float bodyDirZ=getMyBodyAng().z();//rotation angle of body;
	  //relToGlobalTrans.rotationZ(-90);
	  relToGlobalTrans.rotationZ(bodyDirZ);
	  relToGlobalTrans.p()=getMyOrigin();
	  relToGlobalTrans.p().z()=0;
	  return relToGlobalTrans.inverseTransform(destPos);
	}
	/**added by dpf
	 * look above, you konws...^_^
	 * the rel is not  lean rel
	 * */
	math::Vector3f transRelPosToGlobalPos(const Vector3f &destPos){
	  math::TransMatrixf relToGlobalTrans;
	  float bodyDirZ=getMyBodyAng().z();//rotation angle of body;
	  //relToGlobalTrans.rotationZ(-90);
	  relToGlobalTrans.rotationZ(bodyDirZ);
	  relToGlobalTrans.p()=getMyOrigin();
	  relToGlobalTrans.p().z()=0;
	  return relToGlobalTrans.transform(destPos);
	 
	}
	/** 
	 * added by dpf
	 * from lean rel pos to rel pos
	 */
	math::Vector3f transLeanRelPosToRelPos(const Vector3f&destPos){
	  math::TransMatrixf leanRelToRelTrans=getBoneRelTrans(robot::humanoid::Humanoid::TORSO);
	  return leanRelToRelTrans.transform(destPos);
	}
	/**
	 * from rel to lean rel pos
	 */
	math::Vector3f transRelPosToLeanRelPos(const Vector3f&destPos){
	  //rotated trans from rel to lean rel, is leanRelToRelTrans, see our documents for details
	  //this name means, it can transfer the lean rel pos to rel pos, so call it leanRelToRelTrans;
	  //leanRelPos*leanRelToRelTrans=relPos,so relPos*(~leanRelToRelTrans)=leanRelPos;
	  math::TransMatrixf leanRelToRelTrans=getBoneRelTrans(robot::humanoid::Humanoid::TORSO);
	  return leanRelToRelTrans.inverseTransform(destPos);
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
	
	//test by dpf
	math::TransMatrixf calcDpfVisionSensorRotation() const;
	///old function using flags
	///set mMyVisionMatrixUsingFlags, if no enough flags is seen return flase
	bool calcVisionSensorRotationUsingFlags(const Vision&v);
	///the lower function
        math::Matrix3x3f calcVisionSensorRotationUsingFlags(const math::Vector3f& myPos,
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

	//test by dpf
	void setDpfBodyTransMatrix(const math::TransMatrixf& rel){
	  mDpfBodyTransMatrix=rel;
	}
    private:
        unsigned int mSimCycle;

        float mStartSimTime;

        float mLostSimTime;

        float mLastTimeSeeEnoughFlags;

        /** the deque of perception from the server */
        TPerceptionDeque mPerceptions;
        const static unsigned int max_perception_size;

        std::map<unsigned int, math::TransMatrixf> mBoneTrans;
	
	//added by dpf local transfer matrix of jonits. ie. torso is identify matrix,lean rel trans
	std::map<unsigned int, math::TransMatrixf> mBoneLocalTrans;
	//added by dpf rel transfer matrix of joints, rel trans, not lean rel
	std::map<unsigned int, math::TransMatrixf> mBoneRelTrans;

        /////////// Illegal Position ///////
        const static math::Vector3f mIllegalPos;

        /////////// Ball ///////////////
        math::Vector3f mBallGlobalPos;
        //math::Vector2f mBallGlobalPos2DWithRelInfo; //TT
        //math::Vector3f mBallGlobalPos3DWithRelInfo; //TT
        //math::Vector3f mG2RGlobalPos; //terry
        //math::Vector3f mG1RGlobalPos; //terry
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

        /// the matrix of the vision sensor using flags
        math::TransMatrixf mMyVisionMatrixUsingFlags;
	///test by dpf
	math::TransMatrixf mDpfMyVisionMatrix;
	///test by dpf, the body ang Z caculated from vision, init pos is -90 degree
	math::AngDeg mVisionBodyAngZ;
        math::Vector3f mMyGlobalVel;
        math::AngDeg mMyFaceDirection;
        math::Vector3f mMyCenterOfMass;
	math::Vector3f mMyRelCenterOfMass;
        math::Vector3f mMyBodyAng;//torso's rotation angles
        math::Vector3f mMyLastBodyAng; //dubai
        math::Vector3f cameraRot; //vision-me
        math::Vector3f mMyGlobalPos; //vision-me
        math::Vector3f ballLaPol; //jia

        /// the coordination for self motion
        math::TransMatrixf mMyOriginMatrix;//it is just mDpfBodyTransMatrix;
        math::AngDeg mMyBodyDirection;
        math::AngDeg mMyBodyDirWithFlags; //TT
        math::TransMatrixf mDpfBodyTransMatrix;//dpf test, torso global transMatrix
       

        /// the global position of the force center in feet
        math::Vector3f mLeftFootForceCenter;
        math::Vector3f mRightFootForceCenter;
        /// the force of feet, dpf change the coordination to lean rel coordination;
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
       // bool localWalkSign;
        //bool rushToGoalSign;
        //bool isLeftFootKick;
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
	///dpf test acc
	//test acc by dpf, rel
	math::Vector3f mMyRealAccRel;
	//global
	math::Vector3f mMyRealAccGlobal;
	//pos from acc calculate, global
	math::Vector3f mMyAccPosGlobal;
	//velocity from acc-sensor,global
	math::Vector3f mMyAccVelGlobal;
	
        /////////////////////////


    }; //end of class WorldModel

#define WM core::WorldModel::GetSingleton()

} //end of namespace core

#endif // CORE_WORLD_MODEL

