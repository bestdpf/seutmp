/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Vision.cpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#include "Vision.h"
#include "Template.hpp"
#include "../core/WorldModel.h"

namespace perception {

    using namespace std;
    using namespace math;
    using namespace serversetting;

    string Vision::mOurTeamName = "SEU";
    map<string, Vision::FID> Vision::mObjectsVision;
    map<string, Vision::PID> Vision::mPlayerVision;
    map<Vision::FID, Vector3f> Vision::mFlagGlobalPos;

    Vision::Vision() {
        mErrV3f = Vector3f(0, 0, 0);
    }

    bool Vision::update(const sexp_t* sexp) {
        std::string name;
        unsigned int lineNum = 0;
        while (sexp) {
            name = "";
            const sexp_t* t = sexp->list;

            if (SEXP_VALUE == t->ty) {
                parser::SexpParser::parseValue(t, name);
                if ("P" == name) // a player
                {
                    string teamName;
                    t = t->next;
                    if (!parser::SexpParser::parseGivenValue(t, "team", teamName))
                        cerr << "Vision can not get the Player's team" << endl;
                    bool isTeammate = false;
                    if (teamName == mOurTeamName)
                        isTeammate = true; // teammate

                    unsigned int id = 0;
                    t = t->next;
                    if (!parser::SexpParser::parseGivenValue(t, "id", id))
                        cerr << "Vision can not get Player's id" << endl;

                    updatePlayer(isTeammate, id, t->next);
                } else if ("L" == name) {
                    t = t->next;
                    if (mFieldLines[lineNum].update(t)) {
                        //===================TT
                        mLineVector.push_back(Line{mFieldLines[lineNum].p0(), mFieldLines[lineNum].p1()});
                        //=================
                        lineNum++;
                    } else {
                        mFieldLines.erase(lineNum);
                    }
                    //  mPosMap[oid] = calLocalRelPos(mPolMap[oid]);
                    //  return true;
                } else {
                    updateObject(name, t->next);
                }
            }

            sexp = sexp->next;
        }

        //    mIsEnoughFlags = getStaticFlagSet().size() >= 2 ? true : false;
        //     calcLines();
        return true;
    }

    bool Vision::updateObject(const string& name, const sexp_t* sexp) {
        FID oid = getObjectVisionID(name);

        if (oid == MYPOS) {
            bool b = parser::SexpParser::parseValue(sexp, mPolMap[oid]);
            cerr << '@' << WM.getGameTime() << "\nserver pos: " << mPolMap[oid] << endl;
            cout << '@' << WM.getGameTime() << "\nserver pos: \t" << mPolMap[oid].x() << '\t' << mPolMap[oid].y() << '\t' << mPolMap[oid].z() << endl;
            return b;
        } else if (oid == MYTRANS) {
            bool b = parser::SexpParser::parseValue(sexp, mMyDebugMat);
            //cout<<mMyDebugMat<<'@'<<WM.getSimTime()<<endl;
            return b;
        }
//        else if (LINE == oid)////////////////////////////////////////////
//        {
//            return true;
//        }
        else {
            if (parser::SexpParser::parseGivenValue(sexp, "pol", mPolMap[oid])) {
                mPosMap[oid] = calLocalRelPos(mPolMap[oid]); //TT test clean
                return true;
            } else {
                return false;
            }
        }
    }

   bool Vision::updatePlayer(bool isTeammate, unsigned int id, const sexp_t* sexp) {
        TTeamPolMap& teamPol = isTeammate ? mOurPol : mOppPol;
        TPlayerPolMap& playerPol = teamPol[id];

        string name, name1;
        while (sexp) {
            name = "";
            name1 = "";
            const sexp_t* t = sexp->list;
            if (SEXP_VALUE == t->ty) {
                if (!parser::SexpParser::parseValue(t, name))
                    return false;

                if ("head" == name) {
                    const sexp_t* t1 = (t->next)->list;
                    if (!parser::SexpParser::parseValue(t1, name1))
                        return false;

                    if ("pol" == name1) {
                        parser::SexpParser::parseValue(t1->next, playerPol[HEAD]);
                    }
                }
                else if ("rlowerarm" == name) {
                    const sexp_t* t1 = (t->next)->list;
                    if (!parser::SexpParser::parseValue(t1, name1))
                        return false;

                    if ("pol" == name1) {
                        parser::SexpParser::parseValue(t1->next, playerPol[R_HAND]);
                    }
                }
                else if ("llowerarm" == name) {
                    const sexp_t* t1 = (t->next)->list;
                    if (!parser::SexpParser::parseValue(t1, name1))
                        return false;

                    if ("pol" == name1) {
                        parser::SexpParser::parseValue(t1->next, playerPol[L_HAND]);
                    }
                }
                else if ("rfoot" == name) {
                    const sexp_t* t1 = (t->next)->list;
                    if (!parser::SexpParser::parseValue(t1, name1))
                        return false;

                    if ("pol" == name1) {
                        parser::SexpParser::parseValue(t1->next, playerPol[R_FOOT]);
                    }
                }
                else if ("lfoot" == name) {
                    const sexp_t* t1 = (t->next)->list;
                    if (!parser::SexpParser::parseValue(t1, name1))
                        return false;

                    if ("pol" == name1) {
                        parser::SexpParser::parseValue(t1->next, playerPol[L_FOOT]);
                    }
                }
            }

            sexp = sexp->next;
        }

        return true;
    }

    std::ostream & operator<<(std::ostream &stream, const Vision& v) {
        stream << "(Vision \n";

        FOR_EACH(iter, v.mPolMap) {
            stream << "(" << Vision::getObjectVisionName(iter->first) << ' ' << iter->second << ")\n";
        }

        FOR_EACH(i, v.mOurPol) {
            stream << "(PT " << i->first << ' ';

            FOR_EACH(j, i->second) {
                stream << "(" << Vision::getPlayerVisionName(j->first) << ' ' << j->second << ")";
            }
            stream << ")\n";
        }

        FOR_EACH(i, v.mOppPol) {
            stream << "(PO " << i->first << ' ';

            FOR_EACH(j, i->second) {
                stream << "(" << Vision::getPlayerVisionName(j->first) << ' ' << j->second << ")";
            }
            stream << ")\n";
        }

        stream << ')';
        return stream;
    }

    Vector3f Vision::calLocalRelPos(const Vector3f& pol) {
        float c = cosDeg(pol[2]);
        float theta = pol[1] + 90;
        return Vector3f(pol[0] * c * cosDeg(theta),
                pol[0] * c * sinDeg(theta),
                pol[0] * sinDeg(pol[2]));
    }

    /**
     * this is just for soccer, in other environment may need other map
     */
    void Vision::setupObjectsVision(serversetting::TTeamIndex ti) {
        mObjectsVision.clear();

        //flags
        if (serversetting::TI_RIGHT == ti) {
            mObjectsVision["F2R"] = F1L;
            mObjectsVision["F1R"] = F2L;
            mObjectsVision["F2L"] = F1R;
            mObjectsVision["F1L"] = F2R;
            mObjectsVision["G2R"] = G1L;
            mObjectsVision["G2L"] = G1R;
            mObjectsVision["G1R"] = G2L;
            mObjectsVision["G1L"] = G2R;
        } else {
            mObjectsVision["F1L"] = F1L;
            mObjectsVision["F2L"] = F2L;
            mObjectsVision["F1R"] = F1R;
            mObjectsVision["F2R"] = F2R;
            mObjectsVision["G1L"] = G1L;
            mObjectsVision["G1R"] = G1R;
            mObjectsVision["G2L"] = G2L;
            mObjectsVision["G2R"] = G2R;
        }

        //ball
        mObjectsVision["B"] = BALL;

//        //lines
//        mObjectsVision["L"] = LINE;

        //debug info
        mObjectsVision["mypos"] = MYPOS;
        mObjectsVision["myMatrix"] = MYMATRIX;
        mObjectsVision["mytrans"] = MYTRANS;

        mObjectsVision["vision_null"] = FID_NULL;
    }

    void Vision::setupStaticObjectsGlobalPos() {
        mFlagGlobalPos.clear();
        mFlagGlobalPos[Vision::F1L] = Vector3f(-half_field_length, half_field_width, 0);
        mFlagGlobalPos[Vision::F2L] = Vector3f(-half_field_length, -half_field_width, 0);
        mFlagGlobalPos[Vision::F1R] = Vector3f(half_field_length, half_field_width, 0);
        mFlagGlobalPos[Vision::F2R] = Vector3f(half_field_length, -half_field_width, 0);
        mFlagGlobalPos[Vision::G1L] = Vector3f(-half_field_length, half_goal_width, goal_height);
        mFlagGlobalPos[Vision::G2L] = Vector3f(-half_field_length, -half_goal_width, goal_height);
        mFlagGlobalPos[Vision::G1R] = Vector3f(half_field_length, half_goal_width, goal_height);
        mFlagGlobalPos[Vision::G2R] = Vector3f(half_field_length, -half_goal_width, goal_height);
    }

    Vision::FID Vision::getObjectVisionID(const std::string& name) {
        return mObjectsVision[name];
    }

    const std::string& Vision::getObjectVisionName(FID id) {
        return getFirstRefBySecondValue(mObjectsVision, id);
    }

    /** the robot can see 5 parts of other robots:
            head, left hand, right hand, left foot, and right foot.
     */
    void Vision::setupPlayerVision() {
        mPlayerVision.clear();
        mPlayerVision["pol"] = TORSO;
        mPlayerVision["head"] = HEAD;
        /// vision name for nao robot
        mPlayerVision["llowerarm"] = L_HAND;
        mPlayerVision["rlowerarm"] = R_HAND;

        /// vision name for soccerbot robot
        mPlayerVision["lhand"] = L_HAND;
        mPlayerVision["rhand"] = R_HAND;
        mPlayerVision["lfoot"] = L_FOOT;
        mPlayerVision["rfoot"] = R_FOOT;

        mPlayerVision["vision_player_null"] = PID_NULL;
    }

    Vision::PID Vision::getPlayerVisionID(const string& name) {
        return mPlayerVision[name];
    }

    const string& Vision::getPlayerVisionName(Vision::PID id) {
        return getFirstRefBySecondValue(mPlayerVision, id);
    }

    bool Vision::isStaticFlag(FID id) {
        return ( id >= F1L) && (id <= G2R);
    }

    set<Vision::FID> Vision::getStaticFlagSet() const {
        set<FID> sf;

        FOR_EACH(iter, mPolMap) {
            if (isStaticFlag(iter->first)) {
                sf.insert(iter->first);
            }
        }

        return sf;
    }

//    bool Vision::canISeeBall() const //vision-me
//    {
//
//        FOR_EACH(iter, mPolMap) {
//            if (iter->first == BALL)return true;
//        }
//        return false;
//    }
//
//    bool Vision::canISee(FID fid) const //YuRobo
//    {
//        if (mPolMap.end() != mPolMap.find(fid)) {
//            return true;
//        } else {
//
//            return false;
//        }
//    }

    bool Vision::isPossibleVisionSensor(const Vector3f& p) const {
        // a little bigger than the 0.005 because of the percesion of compution
        const static float range = 0.005 + 0.0016;

        FOR_EACH(iter, mPolMap) {
            FID id = iter->first;
            if (isStaticFlag(id)) {
                float d = iter->second[0];
                float minDist2 = pow2(d - range);
                float maxDist2 = pow2(d + range);
                float d2 = (p - getFlagGlobalPos(id)).squareLength();
                if (d2 < minDist2 || d2 > maxDist2) {
                    return false;
                }
            }
        }
        return true;
    }

    float Vision::calcPossibilityVisionSensor(const Vector3f& p) const {
        float poss = 1;
        // const static float range = 0.005;
        const static float sigma = 1 / (-2 * pow2(0.0965));

        FOR_EACH(iter, mPolMap) {
            FID id = iter->first;
            if (isStaticFlag(id)) {
                float vd = iter->second[0];
                float rd = (p - getFlagGlobalPos(id)).length();
                float delta = pow2(int((rd - vd)*100) / vd);
                poss *= exp(sigma * delta);
            }
        }
        return poss;
    }


    /*int Vision::getThingNumIsee() const
    {
            int num = 0;

            FOR_EACH(iter, mPolMap) {
                    if ((iter->first) >= 0 && (iter->first) <= 11) {
                            num++;
                    }
            }
            return num;
    }*/
    //存在误差，导致边框点判断不准确，考虑对于截断点出现两次意味着该点和某存在的位置点重合，对于获取的线信息应该线过滤处理。

    bool Vision::calcLines() {
        //   const float pi = M_PI;
        //        TSegmentPointPolMap mPointsPol;
        //        TSegmentPointMap mPointsXY;
        //        TSegmentPolMap mFieldLines;
        //        TLIDMap mFieldLineIDMap;
        unsigned int pointId = 0;

        TSegmentPolMap::const_iterator iterFL;
        //        if(mFieldLines.size()<=1){
        //            cout<<"only one line"<<endl;
        //            return false;
        //        }
        for (iterFL = mFieldLines.begin();
                iterFL != mFieldLines.end();
                ++iterFL) {
            mPointsPol[pointId] = iterFL->second.p0();
            mPointsXY[pointId].x() = mPointsPol[pointId].x() * cos(mPointsPol[pointId].y() / 180 * M_PI) * cos(mPointsPol[pointId].z() / 180 * M_PI);
            mPointsXY[pointId].y() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].y() / 180 * M_PI) * cos(mPointsPol[pointId].z() / 180 * M_PI);
            mPointsXY[pointId].z() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].z() / 180 * M_PI);
            pointId++;
            mPointsPol[pointId] = iterFL->second.p1();
            mPointsXY[pointId].x() = mPointsPol[pointId].x() * cos(mPointsPol[pointId].y() / 180 * M_PI) * cos(mPointsPol[pointId].z() / 180 * M_PI);
            mPointsXY[pointId].y() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].y() / 180 * M_PI) * cos(mPointsPol[pointId].z() / 180 * M_PI);
            mPointsXY[pointId].z() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].z() / 180 * M_PI);
            pointId++;
        }

        const int maxPointsNum = 1000;

        unsigned int pointLeftFarId = maxPointsNum;
        unsigned int pointRightFarId = maxPointsNum;
        unsigned int LineFrontId = maxPointsNum;
        TSegmentPointPolMap::const_iterator iterP;
        for (iterP = mPointsPol.begin();
                iterP != mPointsPol.end();
                iterP++) {
            if (iterP->second.y() < 60.7 && iterP->second.y() > 59.3) {

                if (pointLeftFarId < maxPointsNum) {
                    if (mPointsPol[pointLeftFarId].x() < iterP->second.x()) {
                        pointLeftFarId = iterP->first;
                    }
                } else {
                    pointLeftFarId = iterP->first;
                }
            }
            if (iterP->second.y()>-60.7 && iterP->second.y()<-59.3) {
                if (pointRightFarId < maxPointsNum) {
                    if (mPointsPol[pointRightFarId].x() < iterP->second.x()) {
                        pointRightFarId = iterP->first;
                    }
                } else {
                    pointRightFarId = iterP->first;
                }
            }
        }

        std::cout << " left " << pointLeftFarId / 2 << "right " << pointRightFarId / 2 << std::endl;

        unsigned int pointLeftFarId1 = pointLeftFarId;

        unsigned int pointRightFarId1 = pointRightFarId;


        if (0 == pointLeftFarId1 % 2) {
            pointLeftFarId1++;
        } else {
            pointLeftFarId1--;
        }

        if (0 == pointRightFarId1 % 2) {
            pointRightFarId1++;
        } else {
            pointRightFarId1--;
        }
        if (comparePoints(mPointsPol[pointLeftFarId1], mPointsPol[pointRightFarId1])) {
            std::cout << "see corner" << endl;
        } else if (comparePoints(mPointsPol[pointLeftFarId1], mPointsPol[pointRightFarId])) {
            std::cout << "see one line" << endl;
        } else {

            for (iterFL = mFieldLines.begin();
                    iterFL != mFieldLines.end();
                    ++iterFL) {
                if ((comparePoints(mPointsPol[pointLeftFarId1], iterFL->second.p0())
                        && comparePoints(mPointsPol[pointRightFarId1], iterFL->second.p1()))
                        || (comparePoints(mPointsPol[pointLeftFarId1], iterFL->second.p1())
                        && comparePoints(mPointsPol[pointRightFarId1], iterFL->second.p0()))
                        ) {
                    LineFrontId = iterFL->first;
                    std::cout << "find line !!!!!!!!!!!!!!!" << LineFrontId << endl;
                    break;
                }
            }
            if (LineFrontId == maxPointsNum) {
                std::cout << "cannot find line !!!!!!!!!!!!!!!" << LineFrontId << endl;
            }
        }
        //pointId =0;
        //        Vector3f beam(-6,0,0);
        //         for (iterFL = mFieldLines.begin();
        //                iterFL != mFieldLines.end();
        //                ++iterFL) {
        //            mPointsPol[pointId] = iterFL->second.p0();
        //          //  mPointsPol[pointId].y() += 90;
        //            mPointsXY[pointId].x() = mPointsPol[pointId].x() * cos(mPointsPol[pointId].y() / 180 *  M_PI) * cos(mPointsPol[pointId].z() / 180 *  M_PI);
        //            mPointsXY[pointId].y() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].y() / 180 *  M_PI) * cos(mPointsPol[pointId].z() / 180 *  M_PI);
        //            mPointsXY[pointId].z() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].z() / 180 *  M_PI);
        //            pointId++;
        //
        //            mPointsPol[pointId] = iterFL->second.p1();
        //        //    mPointsPol[pointId].y() += 90;
        //            mPointsXY[pointId].x() = mPointsPol[pointId].x() * cos(mPointsPol[pointId].y() / 180 *  M_PI) * cos(mPointsPol[pointId].z() / 180 *  M_PI);
        //            mPointsXY[pointId].y() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].y() / 180 * M_PI) * cos(mPointsPol[pointId].z() / 180 *  M_PI);
        //            mPointsXY[pointId].z() = mPointsPol[pointId].x() * sin(mPointsPol[pointId].z() / 180 *  M_PI);
        //            pointId++;
        //        }
        //        TSegmentPointMap::const_iterator iterPx;
        //         for (iterPx =  mPointsXY.begin();
        //                iterPx !=  mPointsXY.end();
        //                iterPx++) {
        //             cout<<iterPx->first<<" "<<iterPx->second + beam<<endl;;
        //
        //         }
        mZerr = 30.0f;
        unsigned int farthestp = calcFarthestPolPoint();
        cout << "farthest point " << farthestp << endl;
        unsigned int longestline = calcLongestLine(farthestp);
        cout << "longest line " << longestline << endl;
        {
            cout << "Y^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << longestline << endl;
            math::AngDeg deg = calcClipAngWithY(longestline);
            cout << "degree " << deg << endl;
            Vector3f corr(0.0f, mZerr, deg);
            rotationFieldPoints(corr);
            bool answer = false;
            unsigned int matchP = longestline * 2;
            if (abs(mPointsPol[matchP].y()) > 57) {
                matchP = longestline * 2 + 1;
            }
            matchP = calcMatchPoint();
            if (matchP > mPointsPol.size()) {
                cout << "cannot find" << endl;
                matchP = longestline * 2;
            }
            for (int mp = P_E1_U_L; mp < P_HL2_D; mp++) {
                answer = matchPoints(matchP, (FPID) mp);
            }
            if (mPointsAnswers.empty()) {
                cout << "answer empty" << endl;
            } else {
                cout << "answer size" << mPointsAnswers.size() << endl;
                cout << "PID" << matchP << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
                printAnswer(matchP);

            }
        }
        mPointsAnswers.clear();
        {
            cout << "X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << longestline << endl;
            math::AngDeg deg = calcClipAngWithX(longestline);
            cout << "degree " << deg << endl;
            //  deg = 119.0f;
            Vector3f corr(0.0f, mZerr, deg);
            rotationFieldPoints(corr);
            bool answer = false;
            unsigned int matchP = longestline * 2;
            if (abs(mPointsPol[matchP].y()) > 57) {
                matchP = longestline * 2 + 1;
            }
            matchP = calcMatchPoint();
            if (matchP > mPointsPol.size()) {
                cout << "cannot find" << endl;
                matchP = longestline * 2;
            }
            for (int mp = P_E1_U_L; mp < P_HL2_D; mp++) {
                answer = matchPoints(matchP, (FPID) mp);
            }
            if (mPointsAnswers.empty()) {
                cout << "answer empty" << endl;
            } else {
                cout << "answer size" << mPointsAnswers.size() << endl;
                cout << "PID" << matchP << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
                printAnswer(matchP);

            }
        }
        return true;

        //       for (iterP = mPointsPol.begin();
        //                iterP != mPointsPol.end();
        //                iterP++) {
        //           mPointsPolDealed[iterP->first] =
        //       }

    }

    unsigned int Vision::calcMatchPoint() {
        bool findFirst = false;
        unsigned int res = mPointsPol.size() + 100;
        TSegmentPointPolMap::const_iterator iterP;
        for (iterP = mPointsPol.begin();
                iterP != mPointsPol.end();
                iterP++) {
            if (abs(iterP->second.y()) < 57) {

                if (!findFirst) {
                    findFirst = true;
                    res = iterP->first;
                } else {
                    if (iterP->second.x() > mPointsPol[res].x()) {
                        res = iterP->first;
                    }
                }
            }
        }
        return res;
    }

    void Vision::printAnswer(unsigned int pid) {
        TFPIDMap::const_iterator ti;
        TTFPIDMap::const_iterator tti;
        Vector3f offsetV;
        for (tti = mPointsAnswers.begin();
                tti != mPointsAnswers.end();
                tti++) {
            cout << "FPID" << tti->first << endl;
            offsetV = mPointsXY[pid] - FIELD2.getFieldPoints()[tti->first];
            cout << "pos  " << offsetV << endl;
            for (ti = tti->second.begin();
                    ti != tti->second.end();
                    ti++) {
                cout << "\t\tpid " << ti->first << "FPID " << ti->second << endl;
            }
        }
    }

    bool Vision::matchPoints(unsigned int origPId, FPID fpId) {
        Vector3f offsetV = mPointsXY[origPId] - FIELD2.getFieldPoints()[fpId];
        TFPIDMap matchMap;
        unsigned int pid = origPId;
        matchMap[pid] = fpId;
        FPID tempFP = P_NULL;
        TSegmentPointPolMap::const_iterator iterP;
        for (iterP = mPointsXY.begin();
                iterP != mPointsXY.end();
                iterP++) {
            tempFP = FIELD2.findSimilarPointID(iterP->second - offsetV);
            if (tempFP == P_NULL) {
                if (abs(mPointsPol[iterP->first].y()) < 57) {
                    cout << "match failed" << endl;
                    return false;
                }
            } else {
                matchMap[iterP->first] = tempFP;
            }
        }
        mPointsAnswers[fpId] = matchMap;
        return true;
    }

    bool Vision::rotationFieldPoints(math::Vector3f corr) {
        mPointsXY.clear();
        //   cout << "Rotate"<<endl;
        TSegmentPointPolMap::const_iterator iterP;
        for (iterP = mPointsPol.begin();
                iterP != mPointsPol.end();
                iterP++) {
            mPointsXY[iterP->first] = rotationPoints(pol2xyz(iterP->second), corr);
            cout << iterP->first << " " << mPointsXY[iterP->first] << " old " << pol2xyz(iterP->second) << endl;
        }
    }

    Vector3f Vision::rotationPoints(const Vector3f& p, Vector3f deg) {
        //const &
        //	float objDistToEye=objPolToVisionSensor.x();
        //	float objAngX=objPolToVisionSensor.y();
        //	float objAngY=objPolToVisionSensor.z();
        //
        //	perception::JointPerception jointPerception=WM.lastPerception().joints();
        //	float neckAngX=jointPerception[0].angle();
        //	float neckAngY=jointPerception[1].angle();

        float c1 = cosDeg(deg.x());
        float s1 = sinDeg(deg.x());
        float c2 = cosDeg(deg.y());
        float s2 = sinDeg(deg.y());
        float c3 = cosDeg(deg.z());
        float s3 = sinDeg(deg.z());
        //     cout <<"DDDDD" << deg <<endl;
        float r1TT[3][3] = {1, 0, 0,
            0, c1, -s1,
            0, s1, c1};
        float r2TT[3][3] = {c2, 0, s2,
            0, 1, 0,
            -s2, 0, c2};

        float r3TT[3][3] = {c3, -s3, 0,
            s3, c3, 0,
            0, 0, 1};
        float objXYZEyeTT[3][1] = {p.x(), p.y(), p.z()};
        //      cout << "xyz "<<p<<endl;
        TMatrix<float, 3, 3 > r1 = r1TT;

        TMatrix<float, 3, 3 > r2 = r2TT;
        TMatrix<float, 3, 3 > r3 = r3TT;
        TMatrix<float, 3, 3 > r = r3 * r2*r1;
        //        cout <<"r0"<< r[0]<<endl;
        //        cout <<"r1"<< r[1]<<endl;
        //        cout <<"r2"<< r[2]<<endl;
        TMatrix<float, 3, 1 > objXYZEye = objXYZEyeTT;
        TMatrix<float, 3, 1 > objXYZ = r*objXYZEye;

        //        cout<<"Z "<<objXYZ[2][0]<<endl;

        return Vector3f(objXYZ[0][0], objXYZ[1][0], objXYZ[2][0]);
    }
    //     Vector3f Vision::rotationPoints(const Vector3f& p, Vector3f deg){
    ////        float objDistToEye=objPolToVisionSensor.x();
    ////	float objAngX=objPolToVisionSensor.y();
    ////	float objAngY=objPolToVisionSensor.z();
    //
    //	perception::JointPerception jointPerception=WM.lastPerception().joints();
    //	float neckAngX=deg.z();
    //	float neckAngY=deg.y();
    //
    //	float c1=cosDeg(neckAngY);
    //	float s1=sinDeg(neckAngY);
    //	float c2=cosDeg(neckAngX);
    //	float s2=sinDeg(neckAngX);
    //	float r1TT[3][3]={1,0,0,
    //					  0,c1,-s1,
    //					  0,s1,c1};
    //	float r2TT[3][3]={c2,s2,0,
    //					  -s2,c2,0,
    //					  0,0,1};
    //	float objXYZEyeTT[3][1]={p.x(),p.y(),p.z()};
    //	TMatrix<float,3,3> r1=r1TT;
    //	TMatrix<float,3,3> r2=r2TT;
    //	TMatrix<float,3,3> r=r2*r1;
    //	TMatrix<float,3,1> objXYZEye=objXYZEyeTT;
    //	TMatrix<float,3,1> objXYZ=r*objXYZEye;
    //
    //	return Vector3f( objXYZ[0][0]*(-1) , objXYZ[1][0], objXYZ[2][0]);
    //    }

    unsigned int Vision::calcFarthestPolPoint() {
        unsigned int res = 0;
        TSegmentPointPolMap::const_iterator iterP;
        for (iterP = mPointsPol.begin();
                iterP != mPointsPol.end();
                iterP++) {
            if (iterP->second.x() > mPointsPol[res].x()) {
                res = iterP->first;
            }
        }
        return res;
    }

    unsigned int Vision::calcLongestLine(unsigned int pid) {
        set<unsigned int> pointSet;
        set<unsigned int> lineSet;
        set<unsigned int>::const_iterator sIter;
        pointSet.insert(pid);
        TSegmentPointPolMap::const_iterator iterP;
        for (iterP = mPointsPol.begin();
                iterP != mPointsPol.end();
                iterP++) {
            if (comparePoints(mPointsPol[pid], iterP->second)) {
                pointSet.insert(iterP->first);
            }
        }
        for (sIter = pointSet.begin(); sIter != pointSet.end(); sIter++) {
            lineSet.insert((*sIter) / 2);
        }
        unsigned int LongestLineID = mFieldLines.size() + 100;
        if (lineSet.empty()) {
            return LongestLineID;
        }
        LongestLineID = *lineSet.begin();
        for (sIter = lineSet.begin(); sIter != lineSet.end(); sIter++) {
            if (mFieldLines[*sIter].length() > mFieldLines[LongestLineID].length()) {
                LongestLineID = *sIter;
            }
        }
        return LongestLineID;
    }

    math::AngDeg Vision::calcClipAngWithX(unsigned int lid) {
        Vector3f err(0.0f, 30.0f, 0.0f);
        Vector3f V(rotationPoints(pol2xyz(mFieldLines[lid].p0()), err) - rotationPoints(pol2xyz(mFieldLines[lid].p1()), err));
        cout << "seev1x " << lid << " " << V << " " << mFieldLines[lid].p0() << endl;
        Vector2f v1(V.x(), V.y());
        Vector2f v2(-1.0f, 0.0f);
        return calClipAng(v2, v1);
    }

    math::AngDeg Vision::calcClipAngWithY(unsigned int lid) {
        Vector3f err(0.0f, 30.0f, 0.0f);
        Vector3f V(rotationPoints(pol2xyz(mFieldLines[lid].p0()), err) - rotationPoints(pol2xyz(mFieldLines[lid].p1()), err));
        Vector2f v1(V.x(), V.y());
        cout << "seev1y " << lid << " " << V << " " << mFieldLines[lid].p0() << endl;
        Vector2f v2(0.0f, 1.0f);
        return calClipAng(v2, v1);
    }

    bool Vision::comparePoints(const math::Vector3f& p1, const math::Vector3f& p2) {

        if (abs(p1.x() - p2.x()) < 0.3 && abs(p1.y() - p2.y()) < 1 && abs(p1.z() - p2.z()) < 0.7) {
            return true;
        } else {
            return false;
        }
    }


} //end of namespace perception

