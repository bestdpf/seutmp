/* 
 * File:   FieldInfo.cpp
 * Author: robocup
 * 
 * Created on 2011年4月26日, 上午10:08
 */

#include "FieldInfo.h"
#include "math/Math.hpp"
namespace perception {

    FieldInfo::FieldInfo() :
    mHalfFieldLength(10.5f),
    mHalfFieldWidth(7.0f),
    mFieldRingRadius(1.8f),
    mPerDegreeOfRing(36.0f),
    mPenaltyLength(1.8f),
    mHalfPenaltyWidth(1.95f) {
        calcPoints();
    };

    bool FieldInfo::calcPoints() {
        for (int i = 0; i < P_FNUM; i++) {
            mFieldPoints[i].z() = 0.0f;
        }
        //mFieldPoints[P_FNUM]
        // corner points
        mFieldPoints[P_E1_U_L].x() = -mHalfFieldLength;
        mFieldPoints[P_E1_U_L].y() = mHalfFieldWidth;
        mFieldPoints[P_E2_D_L].x() = -mHalfFieldLength;
        mFieldPoints[P_E2_D_L].y() = -mHalfFieldWidth;
        mFieldPoints[P_E1_U_R].x() = mHalfFieldLength;
        mFieldPoints[P_E1_U_R].y() = mHalfFieldWidth;
        mFieldPoints[P_E2_D_R].x() = mHalfFieldLength;
        mFieldPoints[P_E2_D_R].y() = -mHalfFieldWidth;

        //penalty points
        mFieldPoints[P_P1_UL_L].x() = -mHalfFieldLength;
        mFieldPoints[P_P1_UL_L].y() = mHalfPenaltyWidth;
        mFieldPoints[P_P2_DL_L].x() = -mHalfFieldLength;
        mFieldPoints[P_P2_DL_L].y() = -mHalfPenaltyWidth;
        mFieldPoints[P_P3_DR_L].x() = -mHalfFieldLength + mPenaltyLength;
        mFieldPoints[P_P3_DR_L].y() = -mHalfPenaltyWidth;
        mFieldPoints[P_P4_UR_L].x() = -mHalfFieldLength + mPenaltyLength;
        mFieldPoints[P_P4_UR_L].y() = mHalfPenaltyWidth;

        mFieldPoints[P_P4_UR_R].x() = mHalfFieldLength;
        mFieldPoints[P_P4_UR_R].y() = mHalfPenaltyWidth;
        mFieldPoints[P_P3_DR_R].x() = mHalfFieldLength;
        mFieldPoints[P_P3_DR_R].y() = -mHalfPenaltyWidth;
        mFieldPoints[P_P2_DL_R].x() = mHalfFieldLength - mPenaltyLength;
        mFieldPoints[P_P2_DL_R].y() = -mHalfPenaltyWidth;
        mFieldPoints[P_P1_UL_R].x() = mHalfFieldLength - mPenaltyLength;
        mFieldPoints[P_P1_UL_R].y() = mHalfPenaltyWidth;

        //halfway line points
        mFieldPoints[P_HL1_U].x() = 0.0f;
        mFieldPoints[P_HL1_U].y() = mHalfFieldWidth;
        mFieldPoints[P_HL2_D].x() = 0.0f;
        mFieldPoints[P_HL2_D].y() = -mHalfFieldWidth;
        //ten pointss of center circle
        //M_PI
        for (int c = P_C1; c <= P_C10; c++) {
            mFieldPoints[c].x() = mFieldRingRadius * cos(mPerDegreeOfRing * c / 180.f * M_PI);
            mFieldPoints[c].y() = mFieldRingRadius * sin(mPerDegreeOfRing * c / 180.f * M_PI);
        }
//         for (int i = 0; i < P_FNUM; i++) {
//            mFieldPoints[i].x() = -mFieldPoints[i].x();
//            mFieldPoints[i].y() = -mFieldPoints[i].y();
//         }
        return true;
    }

    FPID FieldInfo::findSimilarPointID(const math::Vector3f orig) {
        for (int i = P_E1_U_L; i < P_FNUM; i++) {
            if(comparePoints2DXY(mFieldPoints[i],orig)){
                return (FPID)i;
            }
        }
        return P_NULL;
    }

    void FieldInfo::setupFieldInfo(serversetting::TTeamIndex ti){

//        std::cout << "ti&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<< ti <<std::endl;
//        if (serversetting::TI_RIGHT == ti) {
//            std::cout << "ti%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"<< ti <<std::endl;
//            for (int i = 0; i < P_FNUM; i++) {
//            mFieldPoints[i].x() = -mFieldPoints[i].x();
//            mFieldPoints[i].y() = -mFieldPoints[i].y();
//        }
//        }
    }

    bool FieldInfo::comparePoints3DXYZ(const math::Vector3f& p1, const math::Vector3f& p2) {

        if (abs(p1.x() - p2.x()) < 0.2 && abs(p1.y() - p2.y()) < 0.2 && abs(p1.z() - p2.z()) < 0.2) {
            return true;
        } else {
            return false;
        }
    }

    bool FieldInfo::comparePoints2DXY(const math::Vector3f& p1, const math::Vector3f& p2) {
        if (abs(p1.x() - p2.x()) < 0.2 && abs(p1.y() - p2.y()) < 0.2) {
            return true;
        } else {
            return false;
        }
    }

    FieldInfo::~FieldInfo() {
    }

}