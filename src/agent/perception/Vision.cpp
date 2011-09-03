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
                       // mLineVector.push_back(Line{mFieldLines[lineNum].p0(), mFieldLines[lineNum].p1()});
                        //above is not a good code, dpf change it to the below
			Line tmpLine;
			tmpLine.PointAPol=mFieldLines[lineNum].p0();
			tmpLine.PointBPol=mFieldLines[lineNum].p1();
			mLineVector.push_back(tmpLine);
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

} //end of namespace perception

