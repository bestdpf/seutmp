/* 
 * File:   FieldInfo.h
 * Author: robocup
 *
 * Created on 2011年4月26日, 上午10:08
 */

#ifndef FIELDINFO_H
#define	FIELDINFO_H
#include "math/Math.hpp"
#include "Singleton.hpp"
#include "SoccerDefines.h"
namespace perception {
    //Field Point ID

    enum FPID {
        // corner points
        P_E1_U_L = 0,
        P_E2_D_L = 1,
        P_E1_U_R=2,
        P_E2_D_R=3,
        //penalty points
        P_P1_UL_L=4,
        P_P2_DL_L=5,
        P_P3_DR_L=6,
        P_P4_UR_L=7,
        P_P1_UL_R=8,
        P_P2_DL_R=9,
        P_P3_DR_R=10,
        P_P4_UR_R=11,
        //halfway line points
        P_HL1_U=12,
        P_HL2_D=13,
        //ten pointss of center circle
        P_C1=14, //x=0,y=Radius
        P_C2=15,
        P_C3=16,
        P_C4=17,
        P_C5=18,
        P_C6=19,
        P_C7=20,
        P_C8=21,
        P_C9=22,
        P_C10=23,
        //num
        P_FNUM=24,
        //NULL
        P_NULL=25

    };

    class FieldInfo : public Singleton<FieldInfo> {
    public:
        FieldInfo();
        virtual ~FieldInfo();

        const math::Vector3f * getFieldPoints() const {
            return mFieldPoints;
        }
        FPID findSimilarPointID(const math::Vector3f orig);
        bool comparePoints3DXYZ(const math::Vector3f& p1, const math::Vector3f& p2);
        bool comparePoints2DXY(const math::Vector3f& p1, const math::Vector3f& p2);
        void setupFieldInfo(serversetting::TTeamIndex ti);
    private:
        bool calcPoints();
        const float mHalfFieldLength;
        const float mHalfFieldWidth;
        const float mFieldRingRadius;
        const math::AngDeg mPerDegreeOfRing;
        const float mPenaltyLength;
        const float mHalfPenaltyWidth;

        math::Vector3f mFieldPoints[P_FNUM];
    };
#define FIELD2 perception::FieldInfo::GetSingleton()
}
#endif	/* FIELDINFO_H */

