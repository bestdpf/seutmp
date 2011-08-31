/* 
 * File:   Localization.h
 * Author: robocup
 *
 * Created on 2011年5月9日, 上午10:40
 */

#ifndef LOCALIZATION_H
#define	LOCALIZATION_H
#include"perception/FieldInfo.h"
#include"WorldModel.h"
#include "math/Math.hpp"
namespace core {

    using namespace math;
    using namespace perception;

    class Localization {
    public:
        Localization();
        bool update();
        virtual ~Localization();

        typedef std::map<unsigned int, math::Vector3f> TSegmentPointPolMap;
        typedef std::map<unsigned int, math::Vector3f> TSegmentPointMap;
        typedef std::map<unsigned int, perception::SegmentPol> TSegmentPolMap;
        typedef std::map<unsigned int, core::FPID> TFPIDMap;
        typedef std::map<core::FPID, TFPIDMap> TTFPIDMap;
        bool calcLines();
        unsigned int calcFarthestPolPoint();
        unsigned int calcLongestLine(unsigned int pid);
        math::AngDeg calcClipAngWithX(unsigned int lid);
        math::AngDeg calcClipAngWithY(unsigned int lid);

        bool matchPoints(unsigned int origPId, core::FPID fpId);
        void printAnswer(unsigned int pid);
        unsigned int calcMatchPoint();

        static bool comparePoints(const math::Vector3f& p1, const math::Vector3f& p2);
    private:

        bool calcRotationMat();
        math::Vector3f rotationPoints(const math::Vector3f& p);
        bool rotationFieldPoints();
        bool selectAnswer(unsigned int pid);
        TSegmentPointPolMap mPointsPol;
        TSegmentPointPolMap mPointsPolDealed;
        TSegmentPointMap mPointsXY;
        TSegmentPolMap mFieldLines;
        TTFPIDMap mPointsAnswers;

        math::AngDeg mRoll;
        math::AngDeg mPitch;
        math::AngDeg mYaw;
        math::Matrix3x3f mRotationMat;
        math::Vector3f mLastPosition;//(x,y,r)r为于x轴正方向夹角
        math::Vector3f mPosition;
        bool mResetPos;


    };
}

#endif	/* LOCALIZATION_H */

