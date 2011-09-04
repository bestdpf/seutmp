/* 
 * File:   PassModel.cpp
 * Author: robocup
 * 传球的对象位置，传球前可以锁定，踢球后解锁
 * Created on 2011年6月11日, 下午4:21
 */

#include <algorithm>
#include <set>

#include "PassModel.h"
#include "core/WorldModel.h"
namespace core {
    using namespace perception;
    using namespace boost;
    using namespace std;
    using namespace math;
    const Vector3f VisionRobot::mErrPol(-1, -1, -1);

    void VisionRobot::calcInfo() {
        mHaveSeen = true;
        const TransMatrixf& eyeMat = WM.getVisionTrans();

        math::AngDeg pitch = -WM.lastPerception().joints()[1].angle();
        calcRotationMat(0, pitch, 0);
        for (int i = perception::Vision::TORSO; i < perception::Vision::PID_NULL; i++) {
            if (mIsSeen[i]) {

                mBodyPartGlobal[i] = eyeMat.transform(Vision::calLocalRelPos(mBodyPartPol[i]));
                // mBodyPartGlobal[i] = PM.rotationPoints(pol2xyz(mBodyPartPol[i]));
                mBodyPartRel[i] = WM.calObjRelPos2D(mBodyPartPol[i]);
                mBodyPartVisionRel[i] = rotationPoints(pol2xyz(mBodyPartPol[i]));
            } else {
                mBodyPartGlobal[i] = mErrPol;
                mBodyPartRel[i] = mErrPol;
                mBodyPartVisionRel[i] = mErrPol;
            }
        }
        if (WM.getMyUnum() == mUnum) {
            mIsSeen[Vision::HEAD] = true;
            mBodyPartGlobal[Vision::HEAD] = WM.getMyGlobalPos();
            mBodyPartRel[Vision::HEAD].x() = 0.0f;
            mBodyPartRel[Vision::HEAD].y() = 0.0f;
            mBodyPartRel[Vision::HEAD].z() = 0.0f;
            mBodyPartVisionRel[Vision::HEAD] = mBodyPartRel[Vision::HEAD];
        }
        calcHeight();
        calcPos();
        calcIsFacedToMe();
        calcIsFacedToBall();
        calcTimeToBall();
        //        
        //                               Vector3f h = Vision::calLocalRelPos(iterOur->second.find(Vision::HEAD)->second);
        //                        mOurTeammates[iterOur->first].mPolPos = iterOur->second.find(Vision::HEAD)->second;
        //                       mOurTeammates[iterOur->first].mGlobalPos = eyeMat.transform(h);
        //                       mOurTeammates[iterOur->first].mRelPos = WM.calObjRelPos2D(mOurTeammates[iterOur->first].mPolPos);
        //         eyeMat.transform(p)calObjRelPos2D
    }

    bool VisionRobot::calcHeight() {
        const float defaultHeight = 0.51f;
        float buttom = 0.0f;
        mHeight = defaultHeight;
        if (WM.getMyUnum() == mUnum) {
            mHeight = defaultHeight;
            mIsFallen = false;
            return true;
        }

        if (mIsSeen[Vision::L_FOOT] || mIsSeen[Vision::R_FOOT]) {
            if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
                buttom = std::min(mBodyPartVisionRel[Vision::L_FOOT].z(), mBodyPartVisionRel[Vision::R_FOOT].z());
            } else if (mIsSeen[Vision::L_FOOT]) {
                buttom = mBodyPartVisionRel[Vision::L_FOOT].z();
            } else {
                buttom = mBodyPartVisionRel[Vision::R_FOOT].z();
            }

        }
        if (mIsSeen[Vision::HEAD]) {
            mHeight = mBodyPartVisionRel[Vision::HEAD].z() - buttom;
        } else if (mIsSeen[Vision::L_HAND] || mIsSeen[Vision::R_HAND]) {
            float seeHeight = 0;
            if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
                seeHeight = std::min(mBodyPartVisionRel[Vision::L_HAND].z(), mBodyPartVisionRel[Vision::R_HAND].z()) - buttom;
            } else if (mIsSeen[Vision::L_HAND]) {
                seeHeight = mBodyPartVisionRel[Vision::L_HAND].z() - buttom;
            } else {
                seeHeight = mBodyPartVisionRel[Vision::R_HAND].z() - buttom;
            }
            mHeight += seeHeight;
            mHeight = mHeight * 1.5f;
        } else {
            mHeight = defaultHeight;
            return false;
        }
        if (mIsOur && mHaveHeard) {

            if (WM.getSimTime() - mlastHearTime < 0.5f) {
                mIsFallen = mHearFallen;
            }

        }
        if (mHeight < 0.3) {
            mIsFallen = true;
        } else {
            mIsFallen = false;
        }
        return true;
    }

    void VisionRobot::calcPos() {
        if (WM.getMyUnum() == mUnum) {

            mGlobalPos = WM.getMyGlobalPos();
            mGlobalPosAll =  WM.getMyGlobalPos2D();
            mRelPos.x() = 0.0f;
            mRelPos.y() = 0.0f;
            mRelPos.z() = 0.0f;
            mGlobalPos.z() = mHeight / 2;
            mRelPos.z() = mHeight / 2;
            return;

        }
        if (mIsSeen[Vision::HEAD]) {
            mGlobalPos = mBodyPartGlobal[Vision::HEAD];
            mRelPos = mBodyPartRel[Vision::HEAD];
        } else if (mIsSeen[Vision::L_FOOT] || mIsSeen[Vision::R_FOOT]) {
            if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
                mGlobalPos = (mBodyPartGlobal[Vision::L_FOOT] + mBodyPartGlobal[Vision::R_FOOT]) / 2;
                mRelPos = (mBodyPartRel[Vision::L_FOOT] + mBodyPartRel[Vision::R_FOOT]) / 2;
            } else if (mIsSeen[Vision::L_FOOT]) {
                mGlobalPos = mBodyPartGlobal[Vision::L_FOOT];
                mRelPos = mBodyPartGlobal[Vision::L_FOOT];
            } else {
                mGlobalPos = mBodyPartGlobal[Vision::R_FOOT];
                mRelPos = mBodyPartGlobal[Vision::R_FOOT];
            }

        } else if (mIsSeen[Vision::L_HAND] || mIsSeen[Vision::R_HAND]) {
            if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
                mGlobalPos = (mBodyPartGlobal[Vision::L_HAND] + mBodyPartGlobal[Vision::R_HAND]) / 2;
                mRelPos = (mBodyPartRel[Vision::L_HAND] + mBodyPartRel[Vision::R_HAND]) / 2;
            } else if (mIsSeen[Vision::L_HAND]) {
                mGlobalPos = mBodyPartGlobal[Vision::L_HAND];
                mRelPos = mBodyPartGlobal[Vision::L_HAND];
            } else {
                mGlobalPos = mBodyPartGlobal[Vision::R_HAND];
                mRelPos = mBodyPartGlobal[Vision::R_HAND];
            }
        }
        if(!mHaveHeard&&WM.getSimTime() - mlastHearTime > 1.0f){
            mGlobalPosAll.set(mGlobalPos.x(),mGlobalPos.y());
        }
        mGlobalPos.z() = mHeight / 2;
        mRelPos.z() = mHeight / 2;
    }

    void VisionRobot::calcIsFacedToMe() {
        mIsFacedToMe = false;
        if (WM.getMyUnum() == mUnum) {
            return;
        }
        if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_FOOT].y() < mBodyPartPol[Vision::R_FOOT].y());
        } else if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_FOOT].y() < mBodyPartPol[Vision::R_HAND].y());
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_HAND].y() < mBodyPartPol[Vision::R_HAND].y());
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToMe = (mBodyPartPol[Vision::L_HAND].y() < mBodyPartPol[Vision::R_FOOT].y());
        }
    }

    void VisionRobot::calcIsFacedToBall() {
        mIsFacedToBall = false;
        Vector2f ballRel = WM.getBallRelPos2D();
        if (WM.getMyUnum() == mUnum) {
            float ToBallDir = WM.getBallRelPos2D().angle();
            if (ToBallDir > 0) {
                mIsFacedToBall = true;
            } else {
                mIsFacedToBall = false;
            }
            return;
        }
        Vector2f lfRel(mBodyPartRel[Vision::L_FOOT].x(), mBodyPartRel[Vision::L_FOOT].y());
        lfRel = lfRel - ballRel;
        Vector2f rfRel(mBodyPartRel[Vision::R_FOOT].x(), mBodyPartRel[Vision::R_FOOT].y());
        rfRel = rfRel - ballRel;
        Vector2f lhRel(mBodyPartRel[Vision::L_HAND].x(), mBodyPartRel[Vision::L_HAND].y());
        lhRel = lhRel - ballRel;
        Vector2f rhRel(mBodyPartRel[Vision::R_HAND].x(), mBodyPartRel[Vision::R_HAND].y());
        rhRel = rhRel - ballRel;
        if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToBall = (calClipAng(lfRel, rfRel) < 0);
        } else if (mIsSeen[Vision::L_FOOT] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToBall = (calClipAng(lfRel, rhRel) < 0);
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_HAND]) {
            mIsFacedToBall = (calClipAng(lhRel, rhRel) < 0);
        } else if (mIsSeen[Vision::L_HAND] && mIsSeen[Vision::R_FOOT]) {
            mIsFacedToBall = (calClipAng(lhRel, rfRel) < 0);
        }
    }

    void VisionRobot::calcIsWalkedToBall() {
        math::Vector2f globalPos2D(mGlobalPos.x(), mGlobalPos.y());
        math::Vector2f globalVel2D(mGlobalVel.x(), mGlobalVel.y());
        math::AngDeg toBallDir = (WM.getBallGlobalPos2D() - globalPos2D).angle();
        math::AngDeg velDir = globalVel2D.angle();
        if (abs(toBallDir - velDir) < 40) {
            mIsWalkedToBall = true;
        } else {
            mIsWalkedToBall = false;
        }
    }

    bool VisionRobot::calcGlobalVel() {
        mGlobalOffset = mGlobalOffset % HISTORY;
        mHistoryGlobalPos[mGlobalOffset] = mGlobalPos;

        if (mHistoryFull) {
            mGlobalVel = (mGlobalPos - mHistoryGlobalPos[(mGlobalOffset + 1) % HISTORY]) / (HISTORY * 0.02);
        } else {
            if (mGlobalOffset > 0) {
                mGlobalVel = (mGlobalPos - mHistoryGlobalPos[0]) / (mGlobalOffset * 0.02);
            } else {
                mGlobalVel = (mGlobalPos - mHistoryGlobalPos[0]);
            }
            mHistoryFull = true;
        }
        mGlobalOffset++;
        calcWalkDirection();
        calcIsWalkedToBall();

    }

    void VisionRobot::calcWalkDirection() {
        mGlobalWalkDirection = mGlobalVel.angle();
    }

    void VisionRobot::calcTimeToBall() {
        const float speed = 0.4f;
        mTimeToBall = 0.0f;
        math::Vector2f dist;
        if (mHaveHeard && WM.getSimTime() - mlastHearTime < 0.5f) {
            dist = (WM.getBallGlobalPos2D() - mHearGlobalPos);
            mTimeToBall += dist.length() / speed;
        }//        if(mHaveSeen&&WM.getSimTime() - mLastSeeTime<0.1f){
            //        math::Vector2f ballRelPos = WM.getBallRelPos2D();
            //        math::Vector2f relPos(mRelPos.x(), mRelPos.y());
            //         dist = ballRelPos - relPos;
            //        mTimeToBall += dist.length() / speed;
            //        }
            //        else
        else {
            math::Vector2f ballRelPos = WM.getBallRelPos2D();
            math::Vector2f relPos(mRelPos.x(), mRelPos.y());
            dist = ballRelPos - relPos;
            mTimeToBall += dist.length() / speed;
        }
       if (mIsFallen) {
            mTimeToBall += 2.7f;
        }
        if (mIsOur == true) {
            if (mHaveHeard) {
                if (mlastHearTime - mLastSeeTime>-1.0f) {
                    float balldir = (WM.getBallGlobalPos2D() - mHearGlobalPos).angle();
                    mTimeToBall += abs(normalizeAngle(balldir - mHearBodyDirection)) / 180.0f * 3.0f;
               }
            } else {
                if (!mIsFacedToBall) {
                    mTimeToBall += 1.5f;
                }
            }
        } else {
            if (!mIsFacedToBall) {
                mTimeToBall += 1.5f;
            }
        }

        if (mIsOur == true) {
            if (mHaveHeard) {
                if (mlastHearTime - mLastSeeTime>-1.0f) {
                    if (WM.getBallGlobalPos2D().x() < mHearGlobalPos.x()) {
                        mTimeToBall += 2.0f * dist.length();
                    }
                }
            } else if (WM.getBallGlobalPos2D().x() < mGlobalPos.x()) {
                mTimeToBall += 2.0f * dist.length();
            }
        }
    }

    bool VisionRobot::calcRotationMat(math::AngDeg roll, math::AngDeg pitch, math::AngDeg yaw) {

        float c1 = cosDeg(roll);
        float s1 = sinDeg(roll);
        float c2 = cosDeg(pitch);
        float s2 = sinDeg(pitch);
        float c3 = cosDeg(yaw);
        float s3 = sinDeg(yaw);

        mRotationMat[0].set(
                c3*c2, -s3 * c1 + c3 * s2 * s1, s3 * s1 + c3 * s2 * c1);
        mRotationMat[1].set(
                s3*c2, c3 * c1 + s3 * s2 * s1, -c3 * s1 + s3 * s2 * c1);
        mRotationMat[2].set(
                -s2, c2 * s1, c2 * c1);
        return true;
    }

    Vector3f VisionRobot::rotationPoints(const Vector3f& p) {

        TMatrix<float, 3, 1 > objXYZEye;

        objXYZEye[0][0] = p.x();
        objXYZEye[1][0] = p.y();
        objXYZEye[2][0] = p.z();
        TMatrix<float, 3, 1 > objXYZ = mRotationMat*objXYZEye;
        return Vector3f(objXYZ[0][0], objXYZ[1][0], objXYZ[2][0]);
    }

    PassModel::PassModel() {
    }

    //PassModel::PassModel(const PassModel& orig) {
    //}

    PassModel::~PassModel() {
    }

    void PassModel::calcOurFastestToBall() {
        float timeNow = WM.getSimTime();
        TSeenPlayer::const_iterator iterOur;
        mOurFastestID = 0;
        mOurMinTimeToBall = 1000.0f;

        for (iterOur = mOurTeammates.begin();
                iterOur != mOurTeammates.end();
                ++iterOur) {
            if (timeNow - iterOur->second.mLastSeeTime < 2.0f || timeNow - iterOur->second.mlastHearTime < 2.0f) {
                if (mOurMinTimeToBall > iterOur->second.mTimeToBall) {
                    mOurMinTimeToBall = iterOur->second.mTimeToBall;
                    mOurFastestID = iterOur->first;
                }
            }
        }
    }

    void PassModel::calcOppFastestToBall() {
        float timeNow = WM.getSimTime();
        TSeenPlayer::const_iterator iterOur;
        mOppFastestID = 0;
        mOppMinTimeToBall = 1000.0f;

        for (iterOur = mOppTeammates.begin();
                iterOur != mOppTeammates.end();
                ++iterOur) {
            if (timeNow - iterOur->second.mLastSeeTime < 3.0f) {
                if (mOppMinTimeToBall > iterOur->second.mTimeToBall) {
                    mOppMinTimeToBall = iterOur->second.mTimeToBall;
                    mOppFastestID = iterOur->first;
                }
            }
        }
    }

    void PassModel::update() {
        updateByListen();
        shared_ptr<const Vision> vp = WM.lastPerception().vision();

        if (NULL == vp.get()) return;

        const Vision::TTeamPolMap& ourPol = vp->ourPolMap();
        updateTeammates(ourPol, mOurTeammates, true);
        unsigned int myUnum = WM.getMyUnum();
        if (myUnum != 0) {
            mOurTeammates[myUnum].mUnum = myUnum;
            mOurTeammates[myUnum].mIsOur = true;
            mOurTeammates[myUnum].mLastSeeTime = WM.getSimTime();
            mOurTeammates[myUnum].calcInfo();
            mOurTeammates[myUnum].calcGlobalVel();
        }
        const Vision::TTeamPolMap& oppPol = vp->oppPolMap();
        updateTeammates(oppPol, mOppTeammates, false);
        calcOurFastestToBall();
        calcOppFastestToBall();
    }

    void PassModel::updateTeammates(const Vision::TTeamPolMap& teamPol, TSeenPlayer& teammates, bool isOur) {
        float timeNow = WM.getSimTime();
        Vector3f errPol(VisionRobot::mErrPol);
        Vision::TTeamPolMap::const_iterator iterTeammatesPol;
        for (iterTeammatesPol = teamPol.begin();
                iterTeammatesPol != teamPol.end();
                ++iterTeammatesPol) {
            teammates[iterTeammatesPol->first].mLastSeeTime = timeNow;
            teammates[iterTeammatesPol->first].mUnum = iterTeammatesPol->first;
            teammates[iterTeammatesPol->first].mIsOur = isOur;
            //filter  objects which could not be seen
            for (int i = perception::Vision::TORSO; i < perception::Vision::PID_NULL; i++) {
                if (iterTeammatesPol->second.find((perception::Vision::PID)i) != iterTeammatesPol->second.end()) {
                    teammates[iterTeammatesPol->first].mBodyPartPol[i] = (iterTeammatesPol->second.find((perception::Vision::PID)i)->second);
                    teammates[iterTeammatesPol->first].mIsSeen[i] = true;
                } else {
                    teammates[iterTeammatesPol->first].mBodyPartPol[i] = errPol;
                    teammates[iterTeammatesPol->first].mIsSeen[i] = false;
                }
            }
            teammates[iterTeammatesPol->first].calcInfo();

        }
 
        TSeenPlayer::iterator iterAllTeammates;
        for (iterAllTeammates = teammates.begin();
                iterAllTeammates != teammates.end();
                ++iterAllTeammates) {
            if (iterAllTeammates->second.getHaveSeen()) {
                iterAllTeammates->second.calcGlobalVel();
            }
        }

    }

    void PassModel::updateByListen() {
        if (!SHM.IsHeard()) {
            return;
        }
        unsigned int unum = SHM.getHearPlayerID();
        float timeNow = WM.getSimTime();

        mOurTeammates[unum].mHaveHeard = true;
        mOurTeammates[unum].mHearGlobalPos = SHM.getHearPlayerPos();
        mOurTeammates[unum].mHearBodyDirection = SHM.getHearPlayerBodyDirection();
        mOurTeammates[unum].mHearBallPos = SHM.getHearBallPos();
        mOurTeammates[unum].mlastHearTime = timeNow;
        mOurTeammates[unum].mIsFallen = SHM.IsHearPlayerFallen();
        mOurTeammates[unum].mHearFallen = SHM.IsHearPlayerFallen();
        mOurTeammates[unum].mIsOur = true;
        mOurTeammates[unum].calcTimeToBall();
        mOurTeammates[unum].mGlobalPosAll.set(SHM.getHearPlayerPos().x(), SHM.getHearPlayerPos().y());

    }

    bool PassModel::canPass() {
        float timeNow = WM.getSimTime();
        TSeenPlayer::const_iterator iterOpp;
        bool resCanPass = true;
        for (iterOpp = mOppTeammates.begin();
                iterOpp != mOppTeammates.end();
                ++iterOpp) {
            if (timeNow - iterOpp->second.mLastSeeTime < 3.0f) {
                if (iterOpp->second.mTimeToBall < 1.5f) {
                    resCanPass = false;
                    break;
                }
            }
        }
        if (resCanPass) {
            mCanPass = true;
            mCanPassTime = timeNow;
            return true;
        } else {
            if (mCanPass && (timeNow - mCanPassTime < 1.5f)) {
                return true;
            } else {
                mCanPass = false;
                return false;
            }
        }
        //    return true;
    }

    bool PassModel::choosePass() {
        float timeNow = WM.getSimTime();
        TSeenPlayer::const_iterator iterOur;
        mPassSet.clear();
        for (iterOur = mOurTeammates.begin();
                iterOur != mOurTeammates.end();
                ++iterOur) {
            if (iterOur->first == WM.getMyUnum()) {
                continue;
            }
            if (iterOur->second.mGlobalPosAll.x() > WM.getBallGlobalPos().x()) {
                if (!isBlocked(iterOur->first)) {
                    mPassSet.insert(iterOur->first);
                }
            }else if(WM.getBallGlobalPos().x()>half_field_length-penalty_length*0.5
                    &&
                    abs(WM.getBallGlobalPos().y())>half_penalty_width){
               if(iterOur->second.mGlobalPosAll.x()>half_field_length-penalty_length*1.2
                       &&
                       abs(iterOur->second.mGlobalPosAll.y())<half_penalty_width*1.2){
                   if (!isBlocked(iterOur->first)) {
                    mPassSet.insert(iterOur->first);
                }
               }
            }
        }
        if (mPassSet.empty()) {
            mBestPassID = 0;
            return false;
        } else {
            std::set<unsigned int>::iterator setIter;
            mBestPassID = *(mPassSet.begin());
            math::AngDeg minDeg = 1000;
            for (setIter = mPassSet.begin();
                    setIter != mPassSet.end();
                    setIter++) {
                Vector2f dist = mOurTeammates[*setIter].mGlobalPosAll - WM.getMyGlobalPos2D();
                float deg = abs(normalizeAngle(dist.angle() - 90));
                if (minDeg > deg) {
                    mBestPassID = *setIter;
                    minDeg = deg;
                }
            }
            return true;
        }
    }

    bool PassModel::isBlocked(unsigned int ourTeammateID) {
        Vector2f our(mOurTeammates[ourTeammateID].mGlobalPos.x(), mOurTeammates[ourTeammateID].mGlobalPos.y());
        Vector2f ballPos = WM.getBallGlobalPos2D();
        Vector2f dist = (ballPos - our);
        TSeenPlayer::const_iterator iterOpp;

        if (dist.length() > 8.0f) {
            return true;
        }
        for (iterOpp = mOppTeammates.begin();
                iterOpp != mOppTeammates.end();
                ++iterOpp) {

            Vector2f opp(iterOpp->second.mGlobalPos.x(), iterOpp->second.mGlobalPos.y());

            Vector2f dd = ballPos - opp;

            if (abs(dd.angle() - dist.angle()) < 20) {
                if (dd.length() < dist.length()) {
                    return false;
                }
            }
        }
        return false;
    }
}