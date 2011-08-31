/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Vision.h,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#ifndef PERCEPTION_VISION_H
#define PERCEPTION_VISION_H

#include <iostream>
#include "math/Math.hpp"
#include "BasicPerception.h"
#include "SoccerDefines.h"
#include "Template.hpp"
#include "SegmentPol.h"
#include "FieldInfo.h"

namespace perception {

    /*
    (Vision
     (Flag_1_l
      (pol 60.0712 -113.686 -88.7676))
     (Flag_2_l (pol 67.9427 -54.0589 -88.7676))
     (Flag_1_r (pol 51.0738 118.2 88.4958))
     (Flag_2_r (pol 60.1349 48.4581 88.4958))
     (Goal_1_l
      (pol 56.2085 -78.1571 -88.7676))
     (Goal_2_l (pol 55.1738 -85.6196 -88.7676))
     (Goal_1_r (pol 46.4692 75.6266 88.4958))
     (Goal_2_r (pol 45.2122 84.6529 88.4958))
     (Ball
      (pol 7.88731 0.000467641 -0.00346655))
      (P (team UES) (id 2) (head (pol 3.00 0.12 -1.47))
                                               (lhand (pol 3.01 -2.13 -5.09))
                                               (rhand (pol 3.02 2.23 -4.84))
                                               (lfoot (pol 3.04 -0.70 -9.66))
                                               (rfoot (pol 3.04 0.73 -9.70))
                                               ))
     */

    class Vision : public BasicPerception {
    public:

        /// these object can only be sensed as a point (line: 2 points)

        enum FID {
            //flags
            F1L = 0,
            F2L,
            F1R,
            F2R,
            //goal
            G1L,
            G2L,
            G1R,
            G2R,
            //ball
            BALL,

            //lines
            LINE,

            //debug info
            MYPOS,
            MYMATRIX,
            MYTRANS,
            FID_NULL /*< this must be the last entry >*/
        };

        enum LID {
            //end line
            EL_L = 0,
            EL_R,
            //sideline
            SL_U,
            SL_D,
            //plenalty line
            PL_L,
            PL_LU,
            PL_LD,
            PL_R,
            PL_RU,
            PL_RD,
            //halfway line
            HWL,
            //ten lines of center circle
            CCL1,
            CCL2,
            CCL3,
            CCL4,
            CCL5,
            CCL6,
            CCL7,
            CCL8,
            CCL9,
            CCL10
        };
        /// different parts of player can bee sensed

        enum PID {
            TORSO = 0,
            HEAD,
            L_HAND,
            R_HAND,
            L_FOOT,
            R_FOOT,
            PID_NULL /*< this must be the last entry >*/
        };

        //=====================TT
	struct Line{
		math::Vector3f PointAPol;
		math::Vector3f PointBPol;
	};

	std::vector<Line> mLineVector;
	//=====================

        typedef std::map<FID, math::Vector3f> TIDVecMap;


        Vision();

        virtual bool update(const sexp_t* sexp);

        friend std::ostream & operator<<(std::ostream &stream, const Vision& v);

        const math::Vector3f& pol(FID oid) const //TT rewrite
        {
            TIDVecMap::const_iterator iter = mPolMap.find(oid);
            if (iter != mPolMap.end())
                return iter->second;
            else
                return mErrV3f;
            //return math::Vector3f(0,0,0);


            /*int num = getThingNumIsee ();
            if(num>0)
                            return getSecondRefByFirstValue(mPolMap, oid);
            else
                            return(math::Vector3f(0,0,0));*/
        }

        const math::Vector3f& pos(FID oid) const {
            return getSecondRefByFirstValue(mPosMap, oid);
        } //TT test clean

        typedef std::map<PID, math::Vector3f> TPlayerPolMap;
        typedef std::map<unsigned int, TPlayerPolMap> TTeamPolMap;

        typedef std::map<unsigned int, math::Vector3f> TSegmentPointPolMap;
        typedef std::map<unsigned int, math::Vector3f> TSegmentPointMap;
        typedef std::map<unsigned int, perception::SegmentPol> TSegmentPolMap;
        typedef std::map<unsigned int, LID> TLIDMap;
        typedef std::map<unsigned int, perception::FPID> TFPIDMap;
        typedef std::map<perception::FPID, TFPIDMap> TTFPIDMap;

        const TTeamPolMap& ourPolMap() const {
            return mOurPol;
        }

        const TTeamPolMap& oppPolMap() const {
            return mOppPol;
        }

        const TIDVecMap& objectPolMap() const {
            return mPolMap;
        }

        /*const TIDVecMap& objectPosMap() const
        {
                return mPosMap;
        }*/

        static math::Vector3f calLocalRelPos(const math::Vector3f& pol);

        /** get debug information from the server */
        const math::TransMatrixf& getMyDebugMat() const {
            return mMyDebugMat;
        }

        static void setOurTeamName(const std::string& name) {
            mOurTeamName = name;
        }

        /** setup the object id <--> object name */
        static void setupObjectsVision(serversetting::TTeamIndex ti);

        /**
         * setup the static objects' global position according to server setting
         */
        static void setupStaticObjectsGlobalPos();

        static FID getObjectVisionID(const std::string& name);

        static const std::string& getObjectVisionName(FID id);

        static void setupPlayerVision();

        static PID getPlayerVisionID(const std::string& name);

        static const std::string& getPlayerVisionName(PID id);

        static const math::Vector3f& getFlagGlobalPos(FID id) {
            return getSecondRefByFirstValue(mFlagGlobalPos, id);
        }

        /**
         * if the object is the static flag
         *
         * @param id the id of the object
         *
         * @return Boolean
         */
        static bool isStaticFlag(FID id);

        /**
         * generate the set of static flags in current vision
         *
         * @return the set of flag's id
         */
        std::set<FID> getStaticFlagSet() const;

        /**
         * check whether the position is ( possible ) the vision sensor position
         *
         * @param p the checked position
         *
         * @return is there any possibility the p is the vision sensor position
         */
        bool isPossibleVisionSensor(const math::Vector3f& p) const;

        /**
         * calculate the possibility that the point is the vision sensor
         *
         * @param p the checked position
         *
         * @return the possibility
         */
        float calcPossibilityVisionSensor(const math::Vector3f& p) const;

        static bool comparePoints(const math::Vector3f& p1, const math::Vector3f& p2);

        static const std::string& ourTeamName() {
            return mOurTeamName;
        }

//        bool canISeeBall()const;
//
//        bool canISee(FID)const; //YuRobo
//
//        bool isEnoughFlags() const {
//            return mIsEnoughFlags;
//        }//vision-me

        TSegmentPolMap getFieldLines()const{
         return mFieldLines;
        }
        bool calcLines();
        //int getThingNumIsee()const;                                               //graz
        unsigned int calcFarthestPolPoint();
        unsigned int calcLongestLine(unsigned int pid);
        math::AngDeg calcClipAngWithX(unsigned int lid);
        math::AngDeg calcClipAngWithY(unsigned int lid);
        bool rotationFieldPoints(math::Vector3f corr);
        bool matchPoints(unsigned int origPId, FPID fpId);
        void printAnswer(unsigned int pid);
        unsigned int calcMatchPoint();
        math::Vector3f rotationPoints(const math::Vector3f& p, math::Vector3f deg);
    private:

        bool updateObject(const std::string& name, const sexp_t* sexp);
        bool updatePlayer(bool isTeammate, unsigned int id, const sexp_t* sexp);

        // mapping from object id to object's position
        TIDVecMap mPolMap;
        TIDVecMap mPosMap; //TT test clean

        TTeamPolMap mOurPol;
        TTeamPolMap mOppPol;

        TSegmentPointPolMap mPointsPol;
        TSegmentPointPolMap mPointsPolDealed;
        TSegmentPointMap mPointsXY;
        TSegmentPolMap mFieldLines;
        TLIDMap mFieldLineIDMap;
        TTFPIDMap mPointsAnswers;
        float mZerr;
        math::TransMatrixf mMyDebugMat;

//        bool mIsEnoughFlags;

        // cache the team name, so we do not access the OPTS every cycle
        static std::string mOurTeamName;

        /** the map of vision object */
        static std::map<std::string, FID> mObjectsVision;
        static std::map<std::string, PID> mPlayerVision;

        /** the global position of static object */
        static std::map<FID, math::Vector3f> mFlagGlobalPos;

        //bool mIsEnoughFlags;

        math::Vector3f mErrV3f;

    }; //end of class Vision

    std::ostream & operator<<(std::ostream &stream, const Vision& v);

} //end of namespace perception

#endif //PERCEPTION_VISION_H

