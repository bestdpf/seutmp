/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: WorldModel.cpp 2731 2009-03-30 14:26:08Z zyj $
 *
 ****************************************************************************/

#include <fstream>
#include "configuration/Configuration.h"
#include "soccer/TeamPlayer.h"
#include "WorldModel.h"
#include "math/TLine2.hpp"
#include "robot/humanoid/Humanoid.h"
#include "PassModel.h"
#include "SayAndHearModel.h"
#include<time.h>

bool sortByDist(const core::WorldModel::BlockInfo& a, const core::WorldModel::BlockInfo& b) {
    return a.dist < b.dist;
}


namespace core {

    using namespace std;
    using namespace boost;
    using namespace math;
    using namespace serversetting;
    using namespace robot;
    using namespace robot::humanoid;
    using namespace action;
    using namespace perception;

    const unsigned int WorldModel::max_perception_size = 100;
    const math::Vector3f WorldModel::mIllegalPos(100000, 100000, -100000);

    WorldModel::WorldModel() {
        mMyAcc = Vector3f(0.0f, 0.0f, 9.81f);


        //===========================================TT Rel
        mFlagRelInfoMap.clear();
        mFlagRelInfoMap[Vision::F1L] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        mFlagRelInfoMap[Vision::F2L] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        mFlagRelInfoMap[Vision::F1R] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        mFlagRelInfoMap[Vision::F2R] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        mFlagRelInfoMap[Vision::G1L] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        mFlagRelInfoMap[Vision::G2L] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        mFlagRelInfoMap[Vision::G1R] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        mFlagRelInfoMap[Vision::G2R] = ObjectRelInfo{false, Vector2f(0, 0), Vector2f(0, 0)};
        //cout<<"mFlagRelInfoMap.size()="<<mFlagRelInfoMap.size()<<endl;
        //===================================================

        /////////////allen add//////////////
        mOurPlayerNum = 0;
        mOppPlayerNum = 0;
        /////////////////////////
        shared_ptr<Perception> p(new Perception());

        // perception
        mPerceptions.push_back(p);
        mSimCycle = 0;
        mStartSimTime = 0;

        // vision
        Vision::setOurTeamName(OPTS.arg<string > ("teamname"));
        Vision::setupObjectsVision(TI_NULL);
        Vision::setupPlayerVision();
        Vision::setupStaticObjectsGlobalPos();

        // ball
        mBallGlobalPos = Vector3f(0, 0, ball_radius);
        mBallGlobalVel.zero();
        mBallGlobalStopPos = mBallGlobalPos;
        mInterceptBallGlobalPos = mBallGlobalPos;
        mBallAveragePos = mBallGlobalPos;
        nowFT = configuration::Formation::FT_HOME; //////////////////////////////////////////////
        bestZoneX = 8;
        bestZoneY = 2;
        mHearOurFastestToBall = 0;
        isAttacking = false;
        // set up ball filter
        const Vector3f errVar(1, 1, 1);
        for (int i = 0; i < 3; i++) {
            mBallPvekf[i].init(0, 0, sim_step, 0.03f,
                    ball_mass, errVar[i] / sim_step, errVar[i]);
        }

        // game info (score)
        mOurGoal = 0;
        mOppGoal = 0;

        /*BEGIN_ADD_LOG_LAYER(WorldModel)
        ADD_LOG_LAYER("localization")
        ADD_LOG_LAYER("body")
        ADD_LOG_LAYER("Ball")
        ADD_LOG_LAYER("CoM")
        ADD_LOG_LAYER("FRP")
        ADD_LOG_LAYER("Players")
        ADD_LOG_LAYER("hear")
        ADD_LOG_LAYER("vision-me")
        ADD_LOG_LAYER("camera")
        //ADD_LOG_LAYER("Gyro");
        //ADD_LOG_LAYER("TT")
        ADD_LOG_LAYER("YuRobo")
        END_ADD_LOG_LAYER(WorldModel)*/
    }

    WorldModel::~WorldModel() {
    }

     int WorldModel::GetOppCount(Vector2f center)
    {
		int count=0;
		int length=2;
		std::map<unsigned int, math::Vector3f>::const_iterator iter;
		FOR_EACH(iter,mOpponentGlobalPos)
		{
		  Vector3f OppPos=iter->second;
		  bool condition=(OppPos.x()<center.x()+length)&&(OppPos.x()>center.x()-length)&&(OppPos.y()>OppPos.y()-length/2)&&(OppPos.y()<center.y()+length);
		  if(condition)
			count++;
		}
		return count;
		//for(int i=1;i<10;i++)
		//{
		//	Vector2f OppPos=getOppGlobalPos2D(i);
		//	bool condition=(OppPos.x()<center.x()+length)&&(OppPos.x()>center.x()-length)&&(OppPos.y()>OppPos.y()-length/2)&&(OppPos.y()<center.y()+length);

		//	if(condition)
		//		count++;

		//}
		//return count;
    }

bool WorldModel::update(shared_ptr<Perception> p)
{
	if (!updatePerception(p))
		return false;

	//TT REL
	updateObjectRelInfo(); //this info may be used in localization
	buildBlocks();

	updateSelf();
	updateBall();
			SHM.update();
	PM.update();
	updatePlayers();



	LOG_FLUSH;

	mSimCycle++;
	if (1 == mSimCycle) {
		mStartSimTime = getSimTime();
	}

	return true;
}

    bool WorldModel::updatePerception(shared_ptr<Perception> p) {
        if (NULL == p.get()) return false;

        const Perception& lp = lastPerception();

        float deltaTime = p->time().now() - lp.time().now();
        mLostSimTime = deltaTime - sim_step;

        if (deltaTime > sim_step / 2) //TT: don't use "deltaTime>0", because it's a floating number
        {
            //1. update goal(score)
            if (lp.getPlayMode() != serversetting::PM_GOAL_LEFT &&
                    lp.getPlayMode() != serversetting::PM_GOAL_RIGHT) {
                if ((p->getPlayMode() == serversetting::PM_GOAL_LEFT
                        && WM.getOurTeamIndex() == serversetting::TI_LEFT)
                        || (p->getPlayMode() == serversetting::PM_GOAL_RIGHT
                        && WM.getOurTeamIndex() == serversetting::TI_RIGHT)) {
                    //cout<<"[WorldModel] our team got a goal\n";
                    mOurGoal++;
                } else if ((p->getPlayMode() == serversetting::PM_GOAL_RIGHT
                        && WM.getOurTeamIndex() == serversetting::TI_LEFT)
                        || (p->getPlayMode() == serversetting::PM_GOAL_LEFT
                        && WM.getOurTeamIndex() == serversetting::TI_RIGHT)) {
                    //cout<<"[WorldModel] our team lost a goal\n";
                    mOppGoal++;
                }
            }

            //2. TT mark: update joints rate in p by lp
            p->update(lp);

            //3. TT mark: predict new perception by last action
            shared_ptr<const Action> lastAct = AGENT.getLastAction();
            if (0 != lastAct.get()) {
                mPredictedPerception = *(p.get());
                float aveStepTime = getAverageStepTime();
                mPredictedPerception.predict(lastAct, aveStepTime); //TT test remove//////////////////////////////////
            }

            //4. update mPerceptions(deque)
            mPerceptions.push_back(p);
            if (mPerceptions.size() > max_perception_size) {
                mPerceptions.pop_front();
            }

            return true;
        }

        return false;
    }

//    void WorldModel::updateSelf() {
//        //1. calculate the global position and rotation of torso
//        localization();
//        const TransMatrixf& vt = getVisionTrans();
//        mMyFaceDirection = atan2Deg(vt.o().y(), vt.o().x());
//
//
//        //2. forward Kinematics, then we got position and rotation of every body
//        mBoneTrans.clear();
//        map<unsigned int, AngDeg> angles = lastPerception().joints().jointAngles();
//        HUMANOID.forwardKinematics(robot::humanoid::Humanoid::TORSO, getVisionTrans(), angles, mBoneTrans);
//
//
//        //3. calculate the center of mass
//        mMyGlobalVel = mMyCenterOfMass; //temp
//        mMyCenterOfMass = HUMANOID.calcCenterOfMass(mBoneTrans);
//        mMyGlobalVel = (mMyCenterOfMass - mMyGlobalVel) / sim_step; //(new-old)/time
//
//        LOG_RED_SPHERE("CoM", mMyCenterOfMass, 0.01);
//        LOG_RED_SPHERE("CoM", Vector3f(mMyCenterOfMass.x(), mMyCenterOfMass.y(), 0), 0.01);
//        LOG_RED_LINE("CoM", mMyCenterOfMass, Vector3f(mMyCenterOfMass.x(), mMyCenterOfMass.y(), 0));
//
//
//        //4. calculate my body rotation angle
//        mMyBodyAng.x() = vt.rotatedAngX();
//        mMyBodyAng.y() = vt.rotatedAngY();
//        mMyBodyAng.z() = vt.rotatedAngZ();
//        LOG_PRINT_VECTOR3("body", mMyBodyAng);
//
//
//        //5. calculate my center, this position is relative to the support foot
//        mMySupportBone = Humanoid::ILLEGAL;
//        mFeetForce.zero();
//        mFeetForcePoint.zero();
//        const ForceResistance& forceResistance = lastPerception().forceResistance();
//        unsigned int lfid = HUMANOID.getBoneId(Humanoid::L_FOOT);
//        unsigned int rfid = HUMANOID.getBoneId(Humanoid::R_FOOT);
//        if (forceResistance.isTouch(lfid)) {
//            mMySupportBone = Humanoid::L_FOOT;
//            const perception::ForceResistance::FeedBack& lfrp = forceResistance.feedBack(lfid);
//            const TransMatrixf& lfm = getBoneTrans(Humanoid::L_FOOT);
//            mLeftFootForceCenter = lfm.transform(lfrp.pos);
//            LOG_RED_SPHERE("FRP", mLeftFootForceCenter, 0.01);
//            LOG_RED_LINE("FRP", mLeftFootForceCenter, mLeftFootForceCenter + lfrp.force * 0.01);
//            mFeetForce = lfrp.force;
//            mFeetForcePoint = mLeftFootForceCenter;
//        }
//
//        if (forceResistance.isTouch(rfid)) {
//            const perception::ForceResistance::FeedBack& rfrp = forceResistance.feedBack(rfid);
//            const TransMatrixf& rfm = getBoneTrans(Humanoid::R_FOOT);
//            mRightFootForceCenter = rfm.transform(rfrp.pos);
//            LOG_RED_SPHERE("FRP", mRightFootForceCenter, 0.01);
//            LOG_RED_LINE("FRP", mRightFootForceCenter, mRightFootForceCenter + rfrp.force * 0.01);
//            if (Humanoid::ILLEGAL == mMySupportBone) {
//                mMySupportBone = Humanoid::R_FOOT;
//                mFeetForce = rfrp.force;
//                mFeetForcePoint = mRightFootForceCenter;
//            } else {
//                // double support, chose the bigger force foot as support foot
//                float lf = forceResistance.feedBack(lfid).force.length();
//                float rf = forceResistance.feedBack(rfid).force.length();
//                if (lf < rf) {
//                    mMySupportBone = Humanoid::R_FOOT;
//                }
//                mFeetForce += rfrp.force;
//                mFeetForcePoint = (mLeftFootForceCenter * lf + mRightFootForceCenter * rf)
//                        / (lf + rf);
//            }
//
//        }
//
//        if (Humanoid::ILLEGAL != mMySupportBone) {
//            mMyOriginMatrix = getBoneTrans(mMySupportBone);
//            if (Humanoid::L_FOOT == mMySupportBone) {
//                mMyOriginMatrix.transfer(HUMANOID.getFootSupportBias(true));
//            } else if (Humanoid::R_FOOT == mMySupportBone) {
//                mMyOriginMatrix.transfer(HUMANOID.getFootSupportBias(false));
//            }
//
//            LOG_AXES("FRP", mMyOriginMatrix, 0.2);
//        }
//
//
//        //6. set in x-y plane
//        mMyBodyDirection = mMyOriginMatrix.rotatedAngZ();
//        Vector3f posOig = mMyOriginMatrix.pos();
//        if (posOig.z() < 0) // assume the z minimum value is 0
//        {
//            float z = -posOig.z();
//
//            FOR_EACH(iter, mBoneTrans) {
//                iter->second.p().z() += z;
//            }
//            posOig.z() = 0;
//        }
//        mMyOriginMatrix.rotationZ(mMyBodyDirection);
//        mMyOriginMatrix.pos() = posOig;
//        mMyBodyDirection = normalizeAngle(mMyBodyDirection + 90);
//
//
//        //7. log
//
//        FOR_EACH(iter, mBoneTrans) {
//            shared_ptr<const Bone> b = HUMANOID.getBone(iter->first);
//            if (NULL == b.get())
//                continue;
//
//            const robot::device::collider::Collider* c = b->collider();
//            if (NULL == c)
//                continue;
//
//            LOG_BLUE_BOX("body", iter->second, c->size());
//        }
//        LOG_BLUE_LINE("body", getMyGlobalPos(), getMyGlobalPos() + Vector3f(cosDeg(mMyFaceDirection), sinDeg(mMyFaceDirection), 0));
//
//
//        //8. Acc
//        //April, MMXI
//        const Vector3f& newAcc = lastPerception().accelerometer().rate(0);
//        mMyAcc = mMyAcc * 0.9f + newAcc * 0.1f;
//    }

void WorldModel::updateSelf()
{
	// 0. vision info
	shared_ptr<const Vision> vp = lastPerception().vision();
	if (vp.get() != NULL) {
		mLatestV = vp;	//terry
	}
	int flagsNumber = getFlagNumbersISee();


	// 1. calculate the global position and rotation of torso
	localization();
	const TransMatrixf& vt = getVisionTrans();
	mMyFaceDirection = atan2Deg(vt.o().y(), vt.o().x());


	// 2. forward Kinematics, then we got position and rotation of every body
	mBoneTrans.clear();
	map<unsigned int, AngDeg> angles = lastPerception().joints().jointAngles();
	TransMatrixf visionTrans = getVisionTrans(); //vision-me
	if(flagsNumber>=3)
		visionTrans = getVisionTrans();
	HUMANOID.forwardKinematics(robot::humanoid::Humanoid::HEAD, visionTrans, angles, mBoneTrans); //vision-me


	// 3. calculate the center of mass
	mMyCenterOfMass = HUMANOID.calcCenterOfMass(mBoneTrans);


	// 4. there are so many heroes x_x
	//vision-me
	Vector3f tempAng = Vector3f(getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngX(),
								getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngY(),
								getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngZ());
	Vector3f deltaAng = tempAng - mMyLastBodyAng;
	//jia
	Vector3f myRot = getMyGyroRate();
	if( abs(getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngY()) != 90
		 && flagsNumber >= 3
		 && (abs(deltaAng.y()) < 50 || abs(getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngY()) < 45) )
	{
		mMyBodyAng.x() = getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngX();
		mMyBodyAng.y() = getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngY();
		mMyBodyAng.z() = getBoneTrans(humanoid::Humanoid::TORSO).rotatedAngZ();
	}
	else
	{
		mMyBodyAng += calMyBodyRotatedAng(calMyBodyAngRate(myRot));
	}//YuRobo
	//dubai
	mMyLastBodyAng = mMyBodyAng;


	// 5. calculate my center, this position is relative to the support foot
	mMySupportBone = Humanoid::ILLEGAL;
	mFeetForce.zero();
	mFeetForcePoint.zero();
	const ForceResistance& forceResistance = lastPerception().forceResistance();
	unsigned int lfid = HUMANOID.getBoneId(Humanoid::L_FOOT);
	unsigned int rfid = HUMANOID.getBoneId(Humanoid::R_FOOT);
	if (forceResistance.isTouch(lfid))
	{
		mMySupportBone = Humanoid::L_FOOT;
		const perception::ForceResistance::FeedBack& lfrp = forceResistance.feedBack(lfid);
		const TransMatrixf& lfm = getBoneTrans(Humanoid::L_FOOT);
		mLeftFootForceCenter = lfm.transform(lfrp.pos);
		mFeetForce = lfrp.force;
		mFeetForcePoint = mLeftFootForceCenter;
	}

	if (forceResistance.isTouch(rfid))
	{
		const perception::ForceResistance::FeedBack& rfrp = forceResistance.feedBack(rfid);
		const TransMatrixf& rfm = getBoneTrans(Humanoid::R_FOOT);
		mRightFootForceCenter = rfm.transform(rfrp.pos);
		if (Humanoid::ILLEGAL == mMySupportBone)
		{
			mMySupportBone = Humanoid::R_FOOT;
			mFeetForce = rfrp.force;
			mFeetForcePoint = mRightFootForceCenter;
		}
		else
		{
			// double support, choose the bigger force foot as support foot
			float lf = forceResistance.feedBack(lfid).force.length();
			float rf = forceResistance.feedBack(rfid).force.length();
			if (lf < rf) {
				mMySupportBone = Humanoid::R_FOOT;
			}
			mFeetForce += rfrp.force;
			mFeetForcePoint = (mLeftFootForceCenter * lf + mRightFootForceCenter * rf) / (lf + rf);
		}
	}

	//vision-me
	if (Humanoid::ILLEGAL != mMySupportBone && flagsNumber >= 3)
	{
		mMyOriginMatrix = getBoneTrans(mMySupportBone);
		if (Humanoid::L_FOOT == mMySupportBone) {
			mMyOriginMatrix.transfer(HUMANOID.getFootSupportBias(true));
		} else if (Humanoid::R_FOOT == mMySupportBone) {
			mMyOriginMatrix.transfer(HUMANOID.getFootSupportBias(false));
		}
	}

	// set in x-y plane
	mMyBodyDirection = mMyOriginMatrix.rotatedAngZ();
	Vector3f posOig = mMyOriginMatrix.pos();
	if (posOig.z() < 0)
	{
		// assume the z minimum value is 0
		float z = -posOig.z();
		FOR_EACH(iter, mBoneTrans) {
			iter->second.p().z() += z;
		}
		posOig.z() = 0;
	}
	mMyOriginMatrix.rotationZ(mMyBodyDirection);
	mMyOriginMatrix.pos() = posOig;
	mMyBodyDirection = normalizeAngle(mMyBodyDirection + 90);

	//Acc
	//April, MMXI
	const Vector3f& newAcc=lastPerception().accelerometer().rate(0);
	mMyAcc=mMyAcc*0.9f+newAcc*0.1f;

	//TT
	//July, MMXI
	calMyBodyDirWithFlags();
}

void WorldModel::updateBall()
{
	static const float minZ = ball_radius - 0.0018f;

	shared_ptr<const Vision> vp = lastPerception().vision();
	if( NULL==vp.get() || false==canSeeBall() ) return;

	// record the postion of ball before,to calculate the velocity of ball
	Vector3f oldPos = mBallGlobalPos;
	const Vector3f& localRelPosBall = vp->pos(Vision::BALL);
	const Vector3f& localRelPosG2R = vp->pos(Vision::G2R);
	const Vector3f& localRelPosG1R = vp->pos(Vision::G1R);
	ballLaPol = vp->pol(perception::Vision::BALL);

	Vector3f posSee = getVisionTrans().transform(localRelPosBall);
	Vector3f posSeeG2R = getVisionTrans().transform(localRelPosG2R); //terry
	Vector3f posSeeG1R = getVisionTrans().transform(localRelPosG1R); //terry
	Vector3f velSee = (posSee - oldPos) / (3 * sim_step);

	// simple simulation
	Vector3f velSim = mBallGlobalVel;
	Vector3f posSim = mBallGlobalPos;
	predictBall(posSim, velSim);
        //cout << "velSee" <<velSee<<"VelSim" <<  velSim<< endl;
        Vector3f visionError(1, 1, 10);
	Vector3f diffVel = velSee - velSim;
        //  cout << "diff" << diffVel<<endl;
        if (pow2(diffVel.x()) + pow2(diffVel.y()) > 16 / 9.0f)
	{
            //    cout << "Ball"<<"the ball is kicked or moved"<<endl;
            LOG_PRINT("Ball", "the ball is kicked or moved");
		// the ball is kicked or be moved
		mBallGlobalPos = posSee;
		mG2RGlobalPos = posSeeG2R; //terry
		mG1RGlobalPos = posSeeG1R; //terry
		mBallGlobalVel = velSee;
		if (mBallGlobalVel.squareLength() > 2500) {
			LOG_PRINT("Ball", "the ball is moved")
			mBallGlobalVel.zero();
		}
		for (int i = 0; i < 3; i++)
			mBallPvekf[i].setP(visionError[i] / (sim_step * 3), 0, 0, visionError[i]);
	}

	else
	{
            //     cout <<"Ball"<< "use filter"<<endl;
            LOG_PRINT("Ball", "use filter");
		mBallGlobalPos = posSim;
		mBallGlobalVel = velSim;
		for(int i = 0; i < 3; i++)
			mBallPvekf[i].update(mBallGlobalVel[i], mBallGlobalPos[i], velSim[i], sim_step * 3,
								 velSee[i], posSee[i], visionError[i]);
	}

	mBallGlobalPos.z() = max(mBallGlobalPos.z(), minZ);

	posSim = mBallGlobalPos;
	velSim = mBallGlobalVel;

	mMyInterceptBallTime = predictInterceptBall(posSim, velSim, getMyOrigin(), 0.4f, 60);
        //mInterceptBallGlobalPos = posSim;
        mInterceptBallGlobalPos = posSee + velSee * 0.06f;

	while (velSim.squareLength() > 0.01f) {
		predictBall(posSim, velSim); //camera cancel(dangerous)
		posSim.z() = max(posSim.z(), minZ);
	}

	mBallGlobalStopPos = posSim;

	// average position of ball
	mBallAveragePos = (mBallAveragePos * mSimCycle + mBallGlobalPos) / (mSimCycle + 1);

	//TT
	calBallGlobalPos2DWithRelInfo();
}


    void WorldModel::updatePlayers() {
        shared_ptr<const Vision> vp = lastPerception().vision();
        if (NULL == vp.get()) return;

        const TransMatrixf& eyeMat = getVisionTrans();

        // 1. teammates
        mOurPlayerNum = 0;
        const Vision::TTeamPolMap& ourPol = vp->ourPolMap();
        mTeammateGlobalPos.clear();

        FOR_EACH(iter, ourPol) {
            Vector3f p = Vision::calLocalRelPos(iter->second.begin()->second);
            mTeammateGlobalPos[iter->first] = eyeMat.transform(p);
            LOG_BLUE_SPHERE("Players", (mTeammateGlobalPos[iter->first]), 0.25f);
          //  mOurPlayerNum++;
        }

        // update myself information in team, for correcting illegal value
        mTeammateGlobalPos[getMyUnum()] = eyeMat.pos();

        // calculate the fastest teammate to the ball
        float player_speed = 0.4f;
        //  mOurFastestToBall = calFastestIdToBall(mTeammateGlobalPos, player_speed, mOurFastestToBallTime);
        mOurFastestToBall = PM.getOurFastestID();
        mOurFastestToBallTime = PM.getOurMinTimeToBall();
        LOG_PRINTF("Players", "our fastest is %d, time = %.3f", mOurFastestToBall, mOurFastestToBallTime);

        // 2. opponents
        mOppPlayerNum = 0;
        const Vision::TTeamPolMap& oppPol = vp->oppPolMap();
        mOpponentGlobalPos.clear();

        FOR_EACH(iter, oppPol) {
            //  Vector3f p = Vision::calLocalRelPos(iter->second.find(Vision::HEAD)->second);
            Vector3f p = Vision::calLocalRelPos(iter->second.begin()->second);
            mOpponentGlobalPos[iter->first] = eyeMat.transform(p);
            LOG_RED_SPHERE("Players", (mOpponentGlobalPos[iter->first]), 0.25f);
            mOppPlayerNum++;
        }

        //   mOppFastestToBall = calFastestIdToBall(mOpponentGlobalPos, 0.6f, mOppFastestToBallTime);
        mOppFastestToBall = PM.getOppFastestID();
        mOppFastestToBallTime = PM.getOppMinTimeToBall();
        mOurPlayerNum = PM.getOurTeammates().size();
        mOppClosestToMe = calClosestIdToPosition(getMyOrigin(), mOpponentGlobalPos, mOppClosestToMeDist);
    }

    void WorldModel::updateObjectRelInfo() {
        shared_ptr<const Vision> v = lastPerception().vision();
        if (NULL == v.get())
            return;

        //init
        mBallRelInfo.canSee = false;

        FOR_EACH(iter, mFlagRelInfoMap) {
            (iter->second).canSee = false;
        }

        //get map in Vision
        const map<Vision::FID, Vector3f>& objPolMap = v->objectPolMap(); //Vision didn't add lines into this map

        FOR_EACH(iter, objPolMap) {
            Vision::FID fid = iter->first;

            //ball
            if (Vision::BALL == fid) {
                Vector2f oldpos = mBallRelInfo.relPos2D;
                mBallRelInfo.canSee = true;
                mBallRelInfo.relPos2D = calObjRelPos2D(iter->second);
                mBallRelInfo.pol2D = calObjPol2D(mBallRelInfo.relPos2D);
                mBallRelVel2D = (mBallRelInfo.relPos2D - oldpos) / (3 * sim_step);
                /*printf("relPos(%.2f, %.2f)\t\t pol(%.2f, %.0f)\n",
                           mBallRelInfo.relPos2D.x(),mBallRelInfo.relPos2D.y(),
                           mBallRelInfo.pol2D.x(),mBallRelInfo.pol2D.y());*/
                continue;
            }                //flags
            else if (fid == Vision::F1L ||
                    fid == Vision::F2L ||
                    fid == Vision::F1R ||
                    fid == Vision::F2R ||
                    fid == Vision::G1L ||
                    fid == Vision::G2L ||
                    fid == Vision::G1R ||
                    fid == Vision::G2R) {
                mFlagRelInfoMap[fid].canSee = true;
                mFlagRelInfoMap[fid].relPos2D = calObjRelPos2D(iter->second);
                mFlagRelInfoMap[fid].pol2D = calObjPol2D(mFlagRelInfoMap[fid].relPos2D);
                //cout<<"fid= "<<fid<<"\t"<<mFlagRelInfoMap[fid].relPos2D.x()<<'\t'<<mFlagRelInfoMap[fid].relPos2D.y()<<endl;
            }
        }
    }

    Vector2f WorldModel::calObjRelPos2D(const Vector3f& objPolToVisionSensor) {
        float objDistToEye = objPolToVisionSensor.x();
        float objAngX = objPolToVisionSensor.y();
        float objAngY = objPolToVisionSensor.z();

        TMatrix<float, 3, 1 > objPosToEye((float[3][1]) {
            {objDistToEye * cosDeg(objAngY) * sinDeg(objAngX)},
            {objDistToEye * cosDeg(objAngY) * cosDeg(objAngX)},
            {objDistToEye * sinDeg(objAngY)}
        });

        const perception::JointPerception& jointPerception = WM.lastPerception().joints();
        float neckAngZ = jointPerception[0].angle(); //rotate angle around z axis
        float neckAngX = jointPerception[1].angle(); //rotate angke around x axis

        float cx = cosDeg(neckAngX);
        float sx = sinDeg(neckAngX);
        float cz = cosDeg(neckAngZ);
        float sz = sinDeg(neckAngZ);

        TMatrix<float, 3, 3 > rx((float[3][3]) {
            {1, 0, 0},
            {0, cx, -sx},
            {0, sx, cx}
        });

        TMatrix<float, 3, 3 > rz((float[3][3]) {
            {cz, sz, 0},
            {-sz, cz, 0},
            {0, 0, 1}
        });

        TMatrix<float, 3, 3 > r = rz*rx; //rz*ry*rx, there is no angle turned around Y axis
        TMatrix<float, 3, 1 > objPosToBody = r*objPosToEye;
        return Vector2f(objPosToBody[0][0]*(-1), objPosToBody[1][0]);
    }

    Vector2f WorldModel::calObjPol2D(const Vector2f& relPos2D) {
        float x = relPos2D.x();
        float y = relPos2D.y();
        float dist = sqrt(x * x + y * y);
        float ang = -atan2Deg(x, y); //atan2Deg returns -180 to 180 angle-degree

        return Vector2f(dist, ang);
    }

    void WorldModel::buildBlocks() {
        shared_ptr<const Vision> v = lastPerception().vision();
        if (NULL == v.get())
            return;

        //===================================for ball
        if (mBallRelInfo.canSee) {
            mBallBlock.dist = mBallRelInfo.pol2D.x();
            mBallBlock.angC = mBallRelInfo.pol2D.y();
            float theta = asinDeg(0.4f / mBallBlock.dist); //ball block size ////////////////////////////////////
            mBallBlock.angL = mBallBlock.angC + theta;
            mBallBlock.angR = mBallBlock.angC - theta;
        }

        //===================================players' body
        mBlockList.clear();
        list<Vector2f> tempV2fList; //body parts relPos2D list
        Vector2f centerOfCircle;
        float radius;
        float tempFloat;
        Vector2f tempV2f;
        float theta;

        //====================our players
        const Vision::TTeamPolMap& ourPlayersMap = v->ourPolMap();

        FOR_EACH(iterPlayer, ourPlayersMap) {
            if (getMyUnum() == iterPlayer->first) //myself
                continue;

            //========================one player's body
            const Vision::TPlayerPolMap& bodyPartMap = iterPlayer->second;

            //add body parts to the list
            tempV2fList.clear();

            FOR_EACH(iterBodyPart, bodyPartMap) {
                tempV2fList.push_back(calObjRelPos2D(iterBodyPart->second));
            }
            if (tempV2fList.empty())
                continue;

            //calculate the center of circle
            centerOfCircle.set(0, 0);

            FOR_EACH(iterV2f, tempV2fList) {
                centerOfCircle += *iterV2f;
            }
            centerOfCircle /= tempV2fList.size();

            //calculate the radius of circle
            radius = 0;

            FOR_EACH(iterV2f, tempV2fList) {
                tempFloat = ((*iterV2f) - centerOfCircle).length();
                if (tempFloat > radius)
                    radius = tempFloat;
            }

            //build a block
            BlockInfo playerBlock;
            tempV2f = calObjPol2D(centerOfCircle);
            playerBlock.dist = tempV2f.x();
            playerBlock.angC = tempV2f.y();
            theta = asinDeg(radius + 0.4f / playerBlock.dist); /////////////////////////////////////// 0 ?
            playerBlock.angL = playerBlock.angC + theta;
            playerBlock.angR = playerBlock.angC - theta;
            mBlockList.push_back(playerBlock);
        } //loop for our players

        //====================opp players
        const Vision::TTeamPolMap& oppPlayersMap = v->oppPolMap();

        FOR_EACH(iterPlayer, oppPlayersMap) {
            //========================one player's body
            const Vision::TPlayerPolMap& bodyPartMap = iterPlayer->second;

            //add body parts to the list
            tempV2fList.clear();

            FOR_EACH(iterBodyPart, bodyPartMap) {
                tempV2fList.push_back(calObjRelPos2D(iterBodyPart->second));
            }
            if (tempV2fList.empty())
                continue;

            //calculate the center of circle
            centerOfCircle.set(0, 0);

            FOR_EACH(iterV2f, tempV2fList) {
                centerOfCircle += *iterV2f;
            }
            centerOfCircle /= tempV2fList.size();

            //calculate the radius of circle
            radius = 0;

            FOR_EACH(iterV2f, tempV2fList) {
                tempFloat = ((*iterV2f) - centerOfCircle).length();
                if (tempFloat > radius)
                    radius = tempFloat;
            }

            //build a block
            BlockInfo playerBlock;
            tempV2f = calObjPol2D(centerOfCircle);
            playerBlock.dist = tempV2f.x();
            playerBlock.angC = tempV2f.y();
            theta = asinDeg(radius + 0.4f / playerBlock.dist); /////////////////////////////////////// 0 ?
            playerBlock.angL = playerBlock.angC + theta;
            playerBlock.angR = playerBlock.angC - theta;
            mBlockList.push_back(playerBlock);
        } //loop for opp players


        //============================sort by dist
        /*FOR_EACH(iter,mBlockList){
                printf("%.2f\n",iter->dist);
        }*/
        mBlockList.sort(sortByDist);
        /*FOR_EACH(iter,mBlockList){
                printf("\t%.2f\n",iter->dist);
        }*/

    }

    Vector2f WorldModel::calMyGlobalPos2DWithTwoFlags() {
        Vector2f flagRelPos1, flagRelPos2;
        Vector3f flagGlobalPos1, flagGlobalPos2;
        Vision::FID fid1, fid2;
        int i = 0;

        FOR_EACH(iter, mFlagRelInfoMap) {
            if ((iter->second).canSee) {
                if (0 == i) {
                    fid1 = iter->first;
                    flagRelPos1 = (iter->second).relPos2D;
                    flagGlobalPos1 = Vision::getFlagGlobalPos(fid1);
                    i++;
                    continue;
                }
                else if (1 == i) {
                    fid2 = iter->first;
                    flagRelPos2 = (iter->second).relPos2D;
                    flagGlobalPos2 = Vision::getFlagGlobalPos(fid2);
                    i++;
                    break;
                }
            }
        }

        //don't do this, please...
        if (i < 2) {
            //printf("%s\n","========= i<2 ============");
            return getMyGlobalPos2D();
        }

        //calculate
        float a1 = flagRelPos1.x();
        float b1 = flagRelPos1.y();
        float a2 = flagRelPos2.x();
        float b2 = flagRelPos2.y();
        float x1 = flagGlobalPos1.x();
        float y1 = flagGlobalPos1.y();
        float x2 = flagGlobalPos2.x();
        float y2 = flagGlobalPos2.y();
        float x, y;
        float B, C, Delta;

        ///////////////////////////////////////
        //printf("\n\n=======================\n");
        //printf("%.1f\n%.1f\n%.1f\n%.1f\n%.1f\n%.1f\n%.1f\n%.1f\n",a1,b1,a2,b2,x1,y1,x2,y2);
        //////////////////////////////////////

        if ((x1 > 0 && x2 > 0) ||
                (x1 < 0 && x2 < 0)) //front court or back court
        {
            y = 0.5f * ((y1 * y1)-(y2 * y2)-(a1 * a1 + b1 * b1 - a2 * a2 - b2 * b2)) / (y1 - y2);
            B = -2 * x1;
            C = x1 * x1 + (y - y1)*(y - y1) - a1 * a1 - b1*b1;
            Delta = B * B - 4 * C; //TT: now, A=1
            if (Delta < 0.0f) {
                //printf("%s\n","========= Delta<0 a ============");
                return getMyGlobalPos2D();
            }

            if (x1 > 0) //front court
                x = (-B - sqrt(Delta)) / 2;
            else //back court
                x = (-B + sqrt(Delta)) / 2;

            //printf("my cal     : x=%.1f\ty=%.1f\n",x,y);
            //printf("real pos is: x=%.1f\ty=%.1f\n\n\n",getMyGlobalPos().x(),getMyGlobalPos().y());
            return Vector2f(x, y);
        }
        else if ((Vision::F1L == fid1 && Vision::F1R == fid2) ||
                (Vision::F2L == fid1 && Vision::F2R == fid2) ||
                (Vision::G1L == fid1 && Vision::G1R == fid2) ||
                (Vision::G2L == fid1 && Vision::G2R == fid2)) {
            x = 0.5f * ((x1 * x1)-(x2 * x2)-(a1 * a1 + b1 * b1 - a2 * a2 - b2 * b2)) / (x1 - x2);
            B = -2 * y1;
            C = y1 * y1 + (x - x1)*(x - x1) - a1 * a1 - b1*b1;
            Delta = B * B - 4 * C; //TT: now, A=1
            if (Delta < 0.0f) {
                //printf("%s\n","========= Delta<0 b ============");
                return getMyGlobalPos2D();
            }

            if (a1 < a2)
                y = (-B - sqrt(Delta)) / 2;
            else
                y = (-B + sqrt(Delta)) / 2;

            //printf("my cal     : x=%.1f\ty=%.1f\n",x,y);
            //printf("real pos is: x=%.1f\ty=%.1f\n\n\n",getMyGlobalPos().x(),getMyGlobalPos().y());
            return Vector2f(x, y);
        }
        else {
            //printf("%s\n","========= on different lines ============");
            return getMyGlobalPos2D();
        }
    }

void WorldModel::calMyBodyDirWithFlags()
{
	if( NULL==lastPerception().vision().get() || 0==seenFlagsNum() )
		return;

	const Vector2f& myPos=getMyGlobalPos2D();
	Vector2f flagRelPos;
	Vector3f flagGlobalPos;
	Vision::FID fid;
	AngDeg myBodyDir=0;
	AngDeg alpha,beta;

	int flagsNum=0;
	FOR_EACH(iter,mFlagRelInfoMap)
	{
		if((iter->second).canSee)
		{
			flagsNum++;
			fid=iter->first;
			flagRelPos=(iter->second).relPos2D;
			flagGlobalPos=Vision::getFlagGlobalPos(fid);

			alpha= ( Vector2f(flagGlobalPos.x(),flagGlobalPos.y()) - myPos ).angle() ;
			beta=atan2Deg(flagRelPos.x(),flagRelPos.y());
			myBodyDir+=alpha+beta;
		}
	}

	if(0==flagsNum)
		return;

	mMyBodyDirWithFlags= normalizeAngle( myBodyDir/((float)flagsNum) ) ;
}


void WorldModel::calBallGlobalPos2DWithRelInfo()
{
	mBallGlobalPos2DWithRelInfo= transRelPosToGlobalPos( getMyGlobalPos2D() , getBallRelPos2D() );
}

    float WorldModel::getAverageStepTime() const {
        shared_ptr<const Perception> pe = mPerceptions.back();
        shared_ptr<const Perception> pb = mPerceptions.front();
        float time = pe->time().now() - pb->time().now();
        int cycle = mPerceptions.size() - 1;
        return floor(time / cycle / serversetting::sim_step + 0.8f) * serversetting::sim_step;
    }
 void WorldModel::localization() {
        //if there is no vision message, do nothing
        const Vision* v = lastPerception().vision().get();
        if (NULL == v) return;

        //calculate the golbal position
        int flagNumbers = seenFlagsNum();

        if (flagNumbers >= 3) {
            mMyGlobalPos = calcVisionSensorPosition(*v); //vision-me
            mLastTimeSeeEnoughFlags = WM.getSimTime();
        } else if (2 == flagNumbers) {
            //mMyGlobalPos = calcVisionSensorPositionWith2Flags(*v); //gxx
            Vector2f tempV2f = calMyGlobalPos2DWithTwoFlags(); //TT
            mMyGlobalPos = Vector3f(tempV2f.x(), tempV2f.y(), 0.5f); //just about 0.5f
            mLastTimeSeeEnoughFlags = WM.getSimTime();
        }

        mMyVisionMatrix.p() = mMyGlobalPos;

        //calculate the body matrix //vision-me
        if (flagNumbers >= 3) {
            set<Vision::FID>::const_iterator flags = v->getStaticFlagSet().begin();
            Vision::FID f1 = *flags;
            Vision::FID f2 = *(++flags);
            Vision::FID f3 = *(++flags);
            mMyVisionMatrix.R() = calcVisionSensorRotation(mMyGlobalPos, *v, f1, f2, f3);
        }
    }

//    void WorldModel::localization() {
//        // if there is no vision message, do nothing
//        const Vision* v = lastPerception().vision().get();
//        if (NULL == v) return;
//
//        // calculate the golbal position
//        const int flagNumbers = getFlagNumbersISee();
//        if (flagNumbers >= 2) //see enough flags
//        {
//            if (flagNumbers >= 3) {
//                mMyGlobalPos = calcVisionSensorPosition(*v); //vision-me
//            } else {
//                //mMyGlobalPos = calcVisionSensorPositionWith2Flags(*v); //gxx
//                Vector2f tempV2f = calMyGlobalPos2DWithTwoFlags(); //TT
//                mMyGlobalPos = Vector3f(tempV2f.x(), tempV2f.y(), 0.5f); //I don't know...
//            }
//
//            mLastTimeSeeEnoughFlags = WM.getSimTime();
//
//            /*if( timeAfterLastSeeEnoughFlags() > 0.03f ) //TT
//            {
//                    Vector3f deltaPos = mMyGlobalPos - mMyVisionMatrix.p();
//                    mMyGlobalVel = deltaPos / timeAfterLastSeeEnoughFlags();
//            }*/
//        }
//
//        /*if( flagNumbers <= 1 ){
//                mMyGlobalPos = calMyGlobalPosByInte(); //TT: may be wrong
//        }*/
//
//        mMyVisionMatrix.p() = mMyGlobalPos;
//
//        // calculate the body matrix //vision-me
//        if (flagNumbers > 2) {
//            set<Vision::FID>::const_iterator flags = v->getStaticFlagSet().begin();
//            Vision::FID f1 = *flags;
//            Vision::FID f2 = *(++flags);
//            Vision::FID f3 = *(++flags);
//            mMyVisionMatrix.R() = calcVisionSensorRotation(mMyGlobalPos, *v, f1, f2, f3);
//        }
//    }

    Matrix3x3f WorldModel::calcVisionSensorRotation(const Vector3f& myPos, const Vision& v,
            Vision::FID flagA, Vision::FID flagB, Vision::FID flagC) const {
        Vector3f relPosFlagA = Vision::getFlagGlobalPos(flagA) - myPos;
        Vector3f relPosFlagB = Vision::getFlagGlobalPos(flagB) - myPos;
        Vector3f relPosFlagC = Vision::getFlagGlobalPos(flagC) - myPos;
        Matrix3x3f relMat(relPosFlagA,
                relPosFlagB,
                relPosFlagC);
        relMat.inv();
        Vector3f localRelPosFlagA = v.pos(flagA);
        Vector3f localRelPosFlagB = v.pos(flagB);
        Vector3f localRelPosFlagC = v.pos(flagC);
        Matrix3x3f localRelMat(localRelPosFlagA,
                localRelPosFlagB,
                localRelPosFlagC);
        Matrix3x3f rotateMat = relMat * localRelMat;
        rotateMat.transpose();
        return rotateMat;
    }

    Vector3f WorldModel::calcVisionSensorPosition(const Vision& v) {
        Vector3f myPos(0, 0, 0);
        vector<Vector3f> pv;
        set<Vision::FID> flags = v.getStaticFlagSet();

        set<Vision::FID>::const_iterator fa = flags.begin();
        set<Vision::FID>::const_iterator fend = flags.end();
        while (fend != fa) {
            set<Vision::FID>::const_iterator fb = fa;
            ++fb;
            while (fend != fb) {
                set<Vision::FID>::const_iterator fc = fb;
                ++fc;
                while (fend != fc) {
                    Vector3f p = calcVisionPositionWithThreeFlags(v, *fa, *fb, *fc);
                    if (!p.isNan()) {
                        pv.push_back(p);
                    }
                    ++fc;
                }
                ++fb;
            }
            ++fa;
        }

        /*fa = flags.begin();
        while (fend != fa) //vision-me
        {
                float temp = *fa;
                LOG_PRINTF("vision-me", "saw flag ID:%.3f", temp);
                ++fa;
        }*/

        if (!pv.empty()) {

            FOR_EACH(iter, pv) {
                myPos += *iter;
            }
            myPos /= pv.size();
        }

        /*if ((myPos.x() == 0) && (myPos.y() == 0)) {
                myPos = calcVisionSensorPositionWith2Flags(v);
        }*/

        /*if (myPos.z() > 0.6f || myPos.z() < 0.4f) {
                myPos.z() = 0.50f; //set it as a const value
        }*/
        return myPos;
    }

    Vector3f WorldModel::calcVisionSensorPositionWith2Flags(const Vision& v) {
        Vector3f myPos(0, 0, 0);
        vector<Vector3f> pv;
        set<Vision::FID> flags = v.getStaticFlagSet();

        set<Vision::FID>::const_iterator fa = flags.begin();
        set<Vision::FID>::const_iterator fend = flags.end();
        while (fend != fa) {
            set<Vision::FID>::const_iterator fb = fa;
            fb++;
            while (fend != fb) {
                Vector3f p = calcVisionPositionWithTwoFlags(v, *fa, *fb);
                if (!p.isNan()) pv.push_back(p);
                fb++;
            }
            fa++;
        }

        if (!pv.empty()) {

            FOR_EACH(iter, pv) {
                myPos += *iter;
            }
            myPos /= pv.size();
        }

        return myPos;
    }

    /*
    void WorldModel::updateG1L()//YuRobo 更新我方右球门柱信息
    {
            shared_ptr<const Vision> vp = lastPerception().vision();
            if (NULL == vp.get() || !canISee(perception::Vision::G1L)) return;
            flagG1LLaPol = vp->pol(perception::Vision::G1L);//YuRobo
    }

    void WorldModel::updateG2L()//YuRobo 更新我方左球门柱信息
    {
            shared_ptr<const Vision> vp = lastPerception().vision();
            if (NULL == vp.get() || !canISee(perception::Vision::G2L)) return;
            flagG2LLaPol = vp->pol(perception::Vision::G2L);//YuRobo
    }

    void WorldModel::updateG1R()//YuRobo 更新对方左球门柱信息
    {
            shared_ptr<const Vision> vp = lastPerception().vision();
            if (NULL == vp.get() || !canISee(perception::Vision::G1R)) return;
            flagG1RLaPol = vp->pol(perception::Vision::G1R);//YuRobo
    }

    void WorldModel::updateG2R()//YuRobo 更新对方右球门柱信息
    {
            shared_ptr<const Vision> vp = lastPerception().vision();
            if (NULL == vp.get() || !canISee(perception::Vision::G2R)) return;
            flagG2RLaPol = vp->pol(perception::Vision::G2R);//YuRobo
    }
     */

    void WorldModel::predictBall(Vector3f& p, Vector3f& v) const {
        static const float k = exp(-0.03f / ball_mass * sim_step);
        v *= k;
        if (p.z() > ball_radius * 1.5f) {
            v.z() -= acceleration_of_gravity*sim_step;
        }
        p += v*sim_step;
    }

    Vector3f WorldModel::getBallPosToBone() const {
        const TransMatrixf& tempMat = WM.getBoneTrans(robot::humanoid::Humanoid::TORSO);
        return tempMat.inverseTransform(WM.getBallGlobalPos()); //获得球相对于人的位置
    }

    Vector3f WorldModel::getPosToBone(Vector3f pos) const {
        const TransMatrixf& tempMat = WM.getBoneTrans(robot::humanoid::Humanoid::TORSO);
        return tempMat.inverseTransform(pos);

    }

    unsigned int WorldModel::calClosestIdToPosition(const Vector3f& pos,
            const map<unsigned int, Vector3f>& data,
            float& minDist) const {
        if (data.empty()) return 0; // should not happen

        map<unsigned int, Vector3f>::const_iterator iter = data.begin();
        map<unsigned int, Vector3f>::const_iterator endIter = data.end();
        unsigned int minNum = iter->first;
        minDist = (pos - iter->second).squareLength();
        for (++iter; endIter != iter; ++iter) {
            float d = (pos - iter->second).squareLength();
            if (d < minDist) {
                minNum = iter->first;
                minDist = d;
            }
        }
        minDist = sqrt(minDist);
        return minNum;
    }

    unsigned int WorldModel::calFastestIdToBall(const std::map<unsigned int, math::Vector3f>& data,
            float speed,
            float& minTime) {
        if (data.empty()) return 0; // should not happens

        const float maxtime = 60;
        map<unsigned int, Vector3f>::const_iterator iter = data.begin();
        map<unsigned int, Vector3f>::const_iterator endIter = data.end();
        unsigned int minNum = iter->first;
        Vector3f pb = getBallGlobalPos();
        Vector3f vb = getBallGlobalVel();
        minTime = predictInterceptBall(pb, vb, iter->second, speed, maxtime);
        for (++iter; endIter != iter; ++iter) {
            pb = getBallGlobalPos();
            vb = getBallGlobalVel();
            float t = predictInterceptBall(pb, vb, iter->second, speed, maxtime);
            LOG_PRINTF("Players", "player %d intercept ball %.3f", iter->first, t);
            //  cout<<"Players"<<"player " <<iter->first <<"intercept ball" << t << "distance"<< (pb-(iter->second)).length() << "pb" << getBallGlobalPos() << "pl"<<iter->second<< endl;
            if (iter->second.z() < 0.3f) // fallen
            {
                LOG_PRINTF("Players", "player %d is fallen (%.3f)", iter->first, (iter->second.z()));
                //    cout<<"Players"<<"player " <<iter->first <<" is fallen " << (iter->second.z()) <<endl;
                //t += 2;
            }

            if (t < minTime) {
                minNum = iter->first;
                minTime = t;
            }
        }
        return minNum;
    }

    float WorldModel::predictInterceptBall(Vector3f& posBall, Vector3f& velBall, const Vector3f& pos,
            float speed, float maxTime) const {
        float time = 0;
        while (time < maxTime) {
            float dist = sqrt(pow2(posBall.x() - pos.x()) + pow2(posBall.y() - pos.y()));
            if (time * speed > dist) {
                return time;
            }

            if (velBall.squareLength() < 0.01f) {
                // the ball stopped
                return time + dist / speed;
            }

            predictBall(posBall, velBall);
            posBall.z() = max(posBall.z(), ball_radius);

            time += sim_step;
        }
        return -1;
    }

    // =====================================
    // high level judgement
    // =====================================

    bool WorldModel::isGroundBall() const {
        return (abs(WM.getBallGlobalPos().z()) < MAX_POSTION_DIFF);
    }

    bool WorldModel::isStable() const {
        // accodring to rotation-Z, rotation-Y, rotation-X order
        /* | cz*cy          sz*cy          -sy   0 |
           | cz*sy*sx-sz*cx sz*sy*sx+cz*cx cy*sx 0 |
           | cz*sy*cx+sz*sx sz*sy*cx-cz*sx cy*cx 0 |
           | 0              0              0     1 | */
        return ( abs(mMyBodyAng.x()) < 20 && abs(mMyBodyAng.y()) < 20);
    }

    bool WorldModel::isLeftRolling() const {
        return mMyBodyAng.y() < -20 && getMyGyroRate().length() > 2;
    }

    bool WorldModel::isLeftRolled() const {
        return ( abs(mMyBodyAng.x()) < 45 &&
                mMyBodyAng.y() < -70 &&
                getMyGyroRate().length() < 5);
    }

    bool WorldModel::isRightRolling() const {
        return ( mMyBodyAng.y() > 20 &&
                getMyGyroRate().length() > 2);
    }

    bool WorldModel::isRightRolled() const {
        return ( abs(mMyBodyAng.x()) < 45 &&
                mMyBodyAng.y() > 70 &&
                getMyGyroRate().length() < 5);
    }

    bool WorldModel::isDiving() const {
        return ((mMyBodyAng.x()<-20 && mMyBodyAng.x()>-90 && getMyGyroRate().x()<-5) ||
                (mMyBodyAng.x()<-90 && getMyGyroRate().x() > 5));
    }

    bool WorldModel::isDived() const {
        return ( getMyAcc().y() < -6); //TT acc
    }

    bool WorldModel::isDived(AngDeg angX) const {
        return ( mMyBodyAng.x() < -angX);
    }

    bool WorldModel::isLying() const {
        return ((mMyBodyAng.x() > 20 && mMyBodyAng.x() < 90 && getMyGyroRate().x() > 2) ||
                (mMyBodyAng.x() > 90 && getMyGyroRate().x()<-2));
    }

    bool WorldModel::isLied() const {
        return ( getMyAcc().y() > 6); //TT acc
    }

    bool WorldModel::isLied(AngDeg angX) const {
        return ( mMyBodyAng.x() > angX);
    }

    bool WorldModel::isLeftFall() const {
        return ( getMyAcc().x() > 8); //TT: graz is mMyBodyAng.y()<-60
    }

    bool WorldModel::isLeftFall(AngDeg angle) const {
        return ( mMyBodyAng.y() < -angle);
    }

    bool WorldModel::isRightFall() const {
        return ( getMyAcc().x() < -8); //TT: graz is mMyBodyAng.y()>60
    }

    bool WorldModel::isRightFall(AngDeg angle) const {
        return ( mMyBodyAng.y() > angle);
    }

    bool WorldModel::isFall() const {
        return ( isLied() || isDived());
    }

    bool WorldModel::amIFastestToBallOfOurTeam() const {
        return ( mOurFastestToBall == getMyUnum());
    }

    float WorldModel::howIFasterToBallThanOpps() const {
        return mOppFastestToBallTime - mMyInterceptBallTime;
    }

    bool WorldModel::isTouchBall(unsigned int footID) const {
        // We assume that no more than one object touches the specified foot.
        // Of course, this is not accurate, however, very easy to achieve.
        // We must make it more accurate in the future!!

        const ForceResistance& forceResistance = lastPerception().forceResistance();

        if (!forceResistance.isTouch(footID)) {
            return false;
        }

        const Vector3f& posBall = WM.getBallGlobalPos();
        if (/*FRID_LEFT_FOOT*/0 == footID) {
            float d = (mLeftFootForceCenter - posBall).squareLength();
            return d < pow2(ball_radius);
        }

        if (/*FRID_RIGHT_FOOT*/1 == footID) {
            float d = (mRightFootForceCenter - posBall).squareLength();
            return d < pow2(ball_radius);
        }

        return false;
    }

    //return true if opp player is in the ball field

    bool WorldModel::isPlayerInField(const Vector2f& pos, float margin) {
        return (fabs(pos[0]) < (field_length / 2.0f + margin)
                && fabs(pos[1]) < (field_width / 2.0f + margin));
    }

    bool WorldModel::isCloseToGoal() const {
        float size = 1;
        const Vector3f& posMe = getMyGlobalPos();
        float x = abs(posMe.x());
        if (x > half_field_length - size && x < half_field_length + goal_depth + size) {
            float y = abs(posMe.y());
            if (y < half_goal_width + size) {
                return true;
            }
        }
        return false;
    }

    bool WorldModel::isHaveOppInArea(TConvexPolygon<float> area) {

        Vector2f opp2D;
        Vector3f opp;
        for (unsigned int i = 1; i <= 11; i++) {
            opp = getOppGlobalPos(i);
            opp2D = Vector2f(opp[0], opp[1]);
            if (!isPlayerInField(opp2D, 4))
                continue;
            if (area.isInside(opp2D))
                return true;
        }
        return false;
    }

    Vector3f WorldModel::calcVisionPositionWithThreeFlags(const Vector3f& posA, const Vector3f& polA, const Vector3f& lposA,
            const Vector3f& posB, const Vector3f& polB, const Vector3f& lposB,
            const Vector3f& posC, const Vector3f& polC, const Vector3f& lposC) const {
        float dA2 = pow2(polA[0]);
        float dB2 = pow2(polB[0]);
        float dC2 = pow2(polC[0]);

        float cA = dA2 - posA.squareLength();
        float cB = dB2 - posB.squareLength();
        float cC = dC2 - posC.squareLength();

        float A1 = 2.0 * (posA.x() - posB.x());
        float B1 = 2.0 * (posA.y() - posB.y());
        float C1 = 2.0 * (posA.z() - posB.z());
        float D1 = cB - cA;

        float A2 = 2.0 * (posA.x() - posC.x());
        float B2 = 2.0 * (posA.y() - posC.y());
        float C2 = 2.0 * (posA.z() - posC.z());
        float D2 = cC - cA;

        float a = A1 * B2 - A2*B1;
        float bx = B2 * C1 - B1*C2;
        float cx = B2 * D1 - B1*D2;
        float by = A2 * C1 - A1*C2;
        float cy = A2 * D1 - A1*D2;

        float A = bx * bx + by * by + a*a;
        float B = 2 * (-bx * cx + a * posA.x() * bx - by * cy - a * posA.y() * by - a * a * posA.z());
        float C = cx * cx - 2 * a * posA.x() * cx + cy * cy + 2 * a * posA.y() * cy - a * a*cA;

        float delta = sqrt(max(B * B - 4 * A*C, 0.0f));

        Vector3f p1, p2;
        p1.z() = (-B + delta) / (2 * A);
        p2.z() = (-B - delta) / (2 * A);
        // if a == 0 : this only happens that three flags in the line in horizontal plane
        // in current server, this means that x1 == x2 == x3
        if (abs(a) < EPSILON) {
            p1.y() = (D2 - C2 * p1.z()) / B2;
            p2.y() = (D2 - C2 * p2.z()) / B2;

            float d = cA - pow2(p1.z()) + 2 * posA.z() * p1.z() - pow2(p1.y()) + 2 * posA.y() * p1.y();
            float dd = sqrt(pow2(posA.x()) + d);

            if (pow2(p1.z() - p2.z()) + pow2(p1.y() - p2.y()) < EPSILON) {
                // p1.z and p2.z are equal
                p1.x() = posA.x() + dd;
                p2.x() = posA.x() - dd;
            } else {
                p1.x() = posA.x() + dd;
                Vector3f tmp = p1;
                tmp.x() = posA.x() - dd;
                if (abs((p1 - posB).squareLength() - dB2) > abs((tmp - posB).squareLength() - dB2)) {
                    p1.x() = tmp.x();
                }

                d = cA - pow2(p2.z()) + 2 * posA.z() * p2.z() - pow2(p2.y()) + 2 * posA.y() * p2.y();
                dd = sqrt(pow2(posA.x()) + d);
                p2.x() = posA.x() + dd;
                tmp = p2;
                tmp.x() = posA.x() - dd;
                if (abs((p2 - posB).squareLength() - dB2) > abs((tmp - posB).squareLength() - dB2)) {
                    p2.x() = tmp.x();
                }
            }
        } else {
            p1.x() = (cx - bx * p1.z()) / a;
            p1.y() = (-cy + by * p1.z()) / a;
            p2.x() = (cx - bx * p2.z()) / a;
            p2.y() = (-cy + by * p2.z()) / a;
        }

        // eliminate repeated root
        Vector3f globalPlane = (posB - posA).cross(posC - posA);
        float globalAng1 = (posA - p1).dot(globalPlane);

        Vector3f localPlane = (lposB - lposA).cross(lposC - lposA);
        float localAng = lposA.dot(localPlane);

        if (sign(globalAng1) == sign(localAng)) {
            return p1;
        } else {
            return p2;
        }
    }

    Vector3f WorldModel::calcVisionPositionWithThreeFlags(const Vision& v, Vision::FID fA, Vision::FID fB, Vision::FID fC) const {
        return calcVisionPositionWithThreeFlags(Vision::getFlagGlobalPos(fA), v.pol(fA), v.pos(fA),
                Vision::getFlagGlobalPos(fB), v.pol(fB), v.pos(fB),
                Vision::getFlagGlobalPos(fC), v.pol(fC), v.pos(fC));
    }

    Vector3f WorldModel::calcVisionPositionWithTwoFlags(const Vector3f& posA, const Vector3f& polA,
            const Vector3f& posB, const Vector3f& polB) const {
        float d1 = polA.x();
        float d2 = polB.x();
        float z = 0.50f;
        float c1 = d1 * d1 - posA.x() * posA.x() - posA.y() * posA.y()-(posA.z() - z)*(posA.z() - z);
        float c2 = d2 * d2 - posB.x() * posB.x() - posB.y() * posB.y()-(posB.z() - z)*(posB.z() - z);
        float a = (posB.x() - posA.x()) / (posA.y() - posB.y());
        float b = (c2 - c1) / (2 * (posA.y() - posB.y()));

        float A = a * a + 1;
        float B = 2 * a * b - 2 * posA.x() - 2 * a * posA.y();
        float C = b * b - 2 * b * posA.y() - c1;
        float delta = sqrt(max((B * B - 4 * A * C), 0.0f));

        Vector3f MyPos1, MyPos2;
        MyPos1.x() = (-B + delta) / (2 * A);
        MyPos2.x() = (-B - delta) / (2 * A);
        MyPos1.y() = a * MyPos1.x() + b;
        MyPos2.y() = a * MyPos2.x() + b;
        MyPos1.z() = MyPos2.z() = z;

        if (abs(MyPos1.x()) < half_field_length && abs(MyPos1.y()) < half_field_width) {
            return MyPos1;
        } else if (abs(MyPos2.x()) < half_field_length && abs(MyPos2.y()) < half_field_width) {
            return MyPos2;
        } else
            return mMyGlobalPos;
    }

    Vector3f WorldModel::calcVisionPositionWithTwoFlags(const Vision& v, Vision::FID fA, Vision::FID fB) const {
        return calcVisionPositionWithTwoFlags(Vision::getFlagGlobalPos(fA), v.pol(fA),
                Vision::getFlagGlobalPos(fB), v.pol(fB));
    }

    const math::TransMatrixf& WorldModel::getLeftFootTrans() const {
        return getBoneTrans(robot::humanoid::Humanoid::L_FOOT);
    }

    const math::TransMatrixf& WorldModel::getRightFootTrans() const {
        return getBoneTrans(robot::humanoid::Humanoid::R_FOOT);
    }

    bool WorldModel::isBallToOurGoal(void) {
        Vector3f ballVel = WM.getBallGlobalVel();
        Vector2f ballVel2D(ballVel.x(), ballVel.y());
        if (ballVel.length() < 0.01f) return false;
        Vector3f ballPos = WM.getBallGlobalPos();
        Vector3f ballToLeftGoal = Vector3f(-half_field_length, half_goal_width, 0) - ballPos;
        AngDeg angOfBallAndLeftGoal = ballToLeftGoal.angle();
        Vector3f ballToRightGoal = Vector3f(-half_field_length, -half_goal_width, 0) - ballPos;
        AngDeg angOfBallAndRightGoal = ballToRightGoal.angle();
        AngDeg angOfBallVel = ballVel2D.angle();
        return isAngInInterval(angOfBallVel, angOfBallAndLeftGoal, angOfBallAndRightGoal);
    }

    /**
     * @author YuRobo (03/28/2010)
     */
    math::Vector3f WorldModel::calMyBodyAngRate(math::Vector3f myRot) {
        Vector3f lastAng = mMyLastBodyAng; //mMyLastViBodyAng;
        float sinR = sinDeg(lastAng[0]), cosR = cosDeg(lastAng[0]); //R
        float cosP = cosDeg(lastAng[1]), tanP = tanDeg(lastAng[1]); //P

        math::Vector3f mMyAngRate;

        if (cosP == 0) {
            LOG_PRINT("body1", "I`m falled"); // to be continued
        } else {
            //Vector3f myRot = getMyGyroRate();

            //	R = mMyViBodyAngRate[0];
            //	p = mMyViBodyAngRate[1];
            //	Y = mMyViBodyAngRate[2];
            //
            //   |Y`|   |0   sinR/cosP    cosR/cosP|     |wx|
            //   |P`| = |0     cosR         -sinR  |  *  |wy|
            //   |R`|   |1   tanP*sinR    tanP*cosR|     |wz|
            //

            mMyAngRate[2] = myRot[1] * sinR / cosP + myRot[2] * cosR / cosP;
            mMyAngRate[1] = myRot[1] * cosR - myRot[3] * sinR;
            mMyAngRate[0] = myRot[0] + myRot[1] * tanP * sinR + myRot[2] * tanP*cosR;
        }
        return mMyAngRate;
    }

    /**
     * @author YuRobo (03/28/2010)
     */
    math::Vector3f WorldModel::calMyBodyRotatedAng(math::Vector3f MyBodyAngRate) {
        //MyViBodyAngRate = calMyBodyAngRate( myRot );
        float deltaTime = lastPerception().time().now() - (*mPerceptions[mPerceptions.size() - 2]).time().now();
        LOG_PRINTF("body1", "deltaTime:%.3f", deltaTime);
        return MyBodyAngRate * deltaTime;
    }

     void WorldModel::logPrintSeenFlags() {
        char str[10];
        str[9] = '\0';
        int i = 0;
        str[i++] = WM.canSeeFlag(Vision::F1L) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G1L) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G2L) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::F2L) ? '1' : '0';
        str[i++] = ' ';
        str[i++] = WM.canSeeFlag(Vision::F1R) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G1R) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::G2R) ? '1' : '0';
        str[i++] = WM.canSeeFlag(Vision::F2R) ? '1' : '0';

        LOG_PRINTF("flags", "%s", str);
    }



} // namespace core
