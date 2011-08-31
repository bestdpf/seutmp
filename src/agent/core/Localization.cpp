/* 
 * File:   Localization.cpp
 * Author: robocup
 * 
 * Created on 2011年5月9日, 上午10:39
 */

#include "Localization.h"

namespace core {

    Localization::Localization() {
        //     mLastPosition =
        mResetPos = true;
    }

    bool Localization::update() {
        mFieldLines = WM.lastPerception().vision()->getFieldLines();
        mRoll = 0.0f;
        mPitch = -WM.lastPerception().joints()[1].angle();
        // mPitch = 30.0f;
        mYaw = 0.0f;
        mLastPosition = mPosition;
        return true;
    }

    Localization::~Localization() {
    }

    //存在误差，导致边框点判断不准确，考虑对于截断点出现两次意味着该点和某存在的位置点重合，对于获取的线信息应该线过滤处理。

    bool Localization::calcLines() {
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
            pointId++;
            mPointsPol[pointId] = iterFL->second.p1();
            pointId++;
        }

        unsigned int farthestp = calcFarthestPolPoint();
    //    cout << "farthest point " << farthestp << endl;
        unsigned int longestline = calcLongestLine(farthestp);
     //   cout << "walkdirct" <<WM.getMyBodyDirection()<<"FFFFFFFFFFFFFFFFF"<<endl ;
     //   cout << "longest line " << longestline << endl;
        {
    //        cout << "Y^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << longestline << endl;
            mYaw = 0.0f;
            calcRotationMat();
            mYaw = calcClipAngWithY(longestline);
        //    mYaw+=180;
            cout << "ydegree " << mYaw << endl;
            //    Vector3f corr(0.0f, mZerr, deg);
            rotationFieldPoints();
            bool answer = false;
            unsigned int matchP = longestline * 2;
            if (abs(mPointsPol[matchP].y()) > 57) {
                matchP = longestline * 2 + 1;
            }
            matchP = calcMatchPoint();
            if (matchP > mPointsPol.size()) {
         //       cout << "cannot find" << endl;
                matchP = longestline * 2;
            }
            for (int mp = P_E1_U_L; mp < P_HL2_D; mp++) {
                answer = matchPoints(matchP, (FPID) mp);
            }
            selectAnswer(matchP);
            if (mPointsAnswers.empty()) {
         //       cout << "answer empty" << endl;
            } else {
        //        cout << "answer size" << mPointsAnswers.size() << endl;
        //        cout << "PID" << matchP << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
                printAnswer(matchP);

            }
        }
        mPointsAnswers.clear();
        {
        //    cout << "X$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$" << longestline << endl;
            mYaw = 0.0f;
            calcRotationMat();
            mYaw = calcClipAngWithX(longestline);
         //   mYaw+=180;
            cout << "xdegree " << mYaw << endl;
            //  deg = 119.0f;
            //    Vector3f corr(0.0f, mZerr, deg);
            rotationFieldPoints();
            bool answer = false;
            unsigned int matchP = longestline * 2;
            if (abs(mPointsPol[matchP].y()) > 57) {
                matchP = longestline * 2 + 1;
            }
            matchP = calcMatchPoint();
            if (matchP > mPointsPol.size()) {
        //        cout << "cannot find" << endl;
                matchP = longestline * 2;
            }
            for (int mp = P_E1_U_L; mp < P_HL2_D; mp++) {
                answer = matchPoints(matchP, (FPID) mp);
            }
            selectAnswer(matchP);
            if (mPointsAnswers.empty()) {
        //        cout << "answer empty" << endl;
            } else {
          //      cout << "answer size" << mPointsAnswers.size() << endl;
          //      cout << "PID" << matchP << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
                printAnswer(matchP);

            }
        }
        return true;
    }

    unsigned int Localization::calcMatchPoint() {
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

    bool Localization::selectAnswer(unsigned int pid) {
        TFPIDMap::const_iterator ti;
        TTFPIDMap::const_iterator tti;
        Vector3f offsetV;

        for (tti = mPointsAnswers.begin();
                tti != mPointsAnswers.end();
                tti++) {
            //      cout << "FPID" << tti->first << endl;
            offsetV = mPointsXY[pid] - FIELD2.getFieldPoints()[tti->first];

            if (!mResetPos) {
                if (abs(offsetV.x() - mLastPosition.x()) > 0.7 || abs(offsetV.y() - mLastPosition.y()) > 0.7) {
                    mPointsAnswers.erase(tti->first);
                    tti--;
                    continue;
                }
            }
            //                    P_E1_U_L = 0,
            //        P_E2_D_L = 1,
            //        P_E1_U_R=2,
            //        P_E2_D_R=3,
            //            bool eraseTi = false;
            //            if ((!eraseTi)&&WM.getLatestVision()->canISee(Vision::F1L)) {
            //                for (ti = tti->second.begin();
            //                        ti != tti->second.end();
            //                        ti++) {
            //                    // cout << "\t\tpid " << ti->first << "FPID " << ti->second << endl;
            //                    if (comparePoints(mPointsPol[ti->first], WM.getLatestVision()->objectPolMap().find(Vision::F1L)->second)) {
            //                        if(ti->second !=core::P_E1_U_L )
            //                        {
            //                            eraseTi = true;
            //                            break;
            //                        }
            //                    }
            //                }
            //            }
            //            if ((!eraseTi)&&WM.getLatestVision()->canISee(Vision::F2L)) {
            //                for (ti = tti->second.begin();
            //                        ti != tti->second.end();
            //                        ti++) {
            //                    // cout << "\t\tpid " << ti->first << "FPID " << ti->second << endl;
            //                    if (comparePoints(mPointsPol[ti->first], (WM.getLatestVision()->objectPolMap()).find(Vision::F2L)->second)) {
            //                        if(ti->second !=core::P_E2_D_L )
            //                        {
            //                            eraseTi = true;
            //                            break;
            //                        }
            //                    }
            //                }
            //            }
            //            if ((!eraseTi)&&WM.getLatestVision()->canISee(Vision::F1R)) {
            //                for (ti = tti->second.begin();
            //                        ti != tti->second.end();
            //                        ti++) {
            //                 //   Vector3f temp = WM.getLatestVision()->objectPolMap()[Vision::F1R];
            //                    // cout << "\t\tpid " << ti->first << "FPID " << ti->second << endl;
            //                    if (comparePoints(mPointsPol[ti->first], (WM.getLatestVision()->objectPolMap()).find(Vision::F1R)->second)) {
            //                        if(ti->second !=core::P_E1_U_R )
            //                        {
            //                            eraseTi = true;
            //                            break;
            //                        }
            //                    }
            //                }
            //            }
            //            if ((!eraseTi)&&WM.getLatestVision()->canISee(Vision::F2R)) {
            //                for (ti = tti->second.begin();
            //                        ti != tti->second.end();
            //                        ti++) {
            //                    // cout << "\t\tpid " << ti->first << "FPID " << ti->second << endl;
            //                    if (comparePoints(mPointsPol[ti->first], WM.getLatestVision()->objectPolMap().find(Vision::F2R)->second)) {
            //                        if(ti->second !=core::P_E2_D_R )
            //                        {
            //                            eraseTi = true;
            //                            break;
            //                        }
            //                    }
            //                }
            //            }
            //
            //            if(eraseTi){
            //                 mPointsAnswers.erase(tti->first);
            //                tti--;
            //                continue;
            //            }

        }
        if (mPointsAnswers.size() > 1) {
     //       cout << "more answer" << endl;
        }
        if (mPointsAnswers.size() == 1) {
       //     cout << "one answer" << endl;
            Vector3f sum(0.0f, 0.0f, 0.0f);
            int num = mPointsAnswers.begin()->second.size();
            for (ti = mPointsAnswers.begin()->second.begin();
                    ti != mPointsAnswers.begin()->second.end();
                    ti++) {
                if (abs(mPointsPol[ti->first].y()) > 57.0f) {
                    num--;
                    continue;
                }
                sum = sum + mPointsXY[ti->first] - FIELD2.getFieldPoints()[ti->second];

            }
            if (num > 0) {
                mPosition = sum / num;
            } else {
                mPosition = mPointsXY[pid] - FIELD2.getFieldPoints()[mPointsAnswers.begin()->first];
            }


       //     cerr << "line: " << mPosition << endl;
            mResetPos = false;
        } else {
     //       cout << "empty or more answer" << endl;
        }
        return true;
    }

    void Localization::printAnswer(unsigned int pid) {
        TFPIDMap::const_iterator ti;
        TTFPIDMap::const_iterator tti;
        Vector3f offsetV;
        for (tti = mPointsAnswers.begin();
                tti != mPointsAnswers.end();
                tti++) {
           // cout << "FPID" << tti->first << endl;
            offsetV = mPointsXY[pid] - FIELD2.getFieldPoints()[tti->first];
            cout << "pos  \t" << offsetV.x() <<'\t'<< offsetV.y() <<'\t'<< offsetV.z() << endl;
            //            for (ti = tti->second.begin();
            //                    ti != tti->second.end();
            //                    ti++) {
            //                cout << "\t\tpid " << ti->first << "FPID " << ti->second << endl;
            //            }
        }
    }

    bool Localization::matchPoints(unsigned int origPId, FPID fpId) {
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
                    //       cout << "match failed" << endl;
                    return false;
                }
            } else {
                matchMap[iterP->first] = tempFP;
            }
        }
        mPointsAnswers[fpId] = matchMap;
        return true;
    }

    bool Localization::rotationFieldPoints() {
        mPointsXY.clear();
        calcRotationMat();
        //   cout << "Rotate"<<endl;
        TSegmentPointPolMap::const_iterator iterP;
        for (iterP = mPointsPol.begin();
                iterP != mPointsPol.end();
                iterP++) {
            mPointsXY[iterP->first] = rotationPoints(pol2xyz(iterP->second));
            //        cout << iterP->first << " " << mPointsXY[iterP->first] << " old " << pol2xyz(iterP->second) << endl;
        }
    }

    bool Localization::calcRotationMat() {

        float c1 = cosDeg(mRoll);
        float s1 = sinDeg(mRoll);
        float c2 = cosDeg(mPitch);
        float s2 = sinDeg(mPitch);
        float c3 = cosDeg(mYaw);
        float s3 = sinDeg(mYaw);
   //     cout << " " << mRoll << " " << mPitch << " " << mYaw;
        //        float r1TT[3][3] = {1, 0, 0,
        //            0, c1, -s1,
        //            0, s1, c1};
        //        float r2TT[3][3] = {c2, 0, s2,
        //            0, 1, 0,
        //            -s2, 0, c2};
        //
        //        float r3TT[3][3] = {c3, -s3, 0,
        //            s3, c3, 0,
        //            0, 0, 1};
        //   float objXYZEyeTT[3][1] = {p.x(),p.y(),p.z()};
        //   cout << "xyz "<<p<<endl;
        //        TMatrix<float, 3, 3 > r1 = r1TT;
        //
        //        TMatrix<float, 3, 3 > r2 = r2TT;
        //        TMatrix<float, 3, 3 > r3 = r3TT;
        //        TMatrix<float, 3, 3 > r = r3 * r2*r1;
        //        cout << "r0" << r[0] << endl;
        //        cout << "r1" << r[1] << endl;
        //        cout << "r2" << r[2] << endl;
        mRotationMat[0].set(
                c3*c2, -s3 * c1 + c3 * s2 * s1, s3 * s1 + c3 * s2 * c1);
        mRotationMat[1].set(
                s3*c2, c3 * c1 + s3 * s2 * s1, -c3 * s1 + s3 * s2 * c1);
        mRotationMat[2].set(
                -s2, c2 * s1, c2 * c1);
        //        cout << "mRotationMat0" << mRotationMat[0] << endl;
        //        cout << "mRotationMat1" << mRotationMat[1] << endl;
        //        cout << "mRotationMat2" << mRotationMat[2] << endl;
        return true;
    };

    Vector3f Localization::rotationPoints(const Vector3f& p) {

        //         float objXYZEyeTT[3][1] = {p.x(),p.y(),p.z()};
        //          TMatrix<float, 3, 1 > objXYZEye = objXYZEyeTT;
        TMatrix<float, 3, 1 > objXYZEye;

        objXYZEye[0][0] = p.x();
        objXYZEye[1][0] = p.y();
        objXYZEye[2][0] = p.z();
        TMatrix<float, 3, 1 > objXYZ = mRotationMat*objXYZEye;
        //     cout << "Z " << objXYZ[2][0] << endl;
        return Vector3f(objXYZ[0][0], objXYZ[1][0], objXYZ[2][0]);
    }
    //     Vector3f Localization::rotationPoints(const Vector3f& p, Vector3f deg){
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

    unsigned int Localization::calcFarthestPolPoint() {
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

    unsigned int Localization::calcLongestLine(unsigned int pid) {
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

    math::AngDeg Localization::calcClipAngWithX(unsigned int lid) {
        //  Vector3f err(0.0f, 30.0f, 0.0f);
        Vector3f V(rotationPoints(pol2xyz(mFieldLines[lid].p0())) - rotationPoints(pol2xyz(mFieldLines[lid].p1())));
        //      cout << "lov1x " << lid << " " << V << " " << mFieldLines[lid].p0() << endl;
        Vector2f v1(V.x(), V.y());
        Vector2f v2(-1.0f, 0.0f);
        return calClipAng(v2, v1);
    }

    math::AngDeg Localization::calcClipAngWithY(unsigned int lid) {
        //   Vector3f err(0.0f, 30.0f, 0.0f);
        Vector3f V(rotationPoints(pol2xyz(mFieldLines[lid].p0())) - rotationPoints(pol2xyz(mFieldLines[lid].p1())));
        Vector2f v1(V.x(), V.y());
        //    cout << "lov1y " << lid << " " << V << " " << mFieldLines[lid].p0() << endl;
        Vector2f v2(0.0f, 1.0f);
        return calClipAng(v2, v1);
    }

    bool Localization::comparePoints(const math::Vector3f& p1, const math::Vector3f& p2) {

        if (abs(p1.x() - p2.x()) < 0.3 && abs(p1.y() - p2.y()) < 1 && abs(p1.z() - p2.z()) < 0.7) {
            return true;
        } else {
            return false;
        }
    }

}
