/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Hear.h"
#include "core/WorldModel.h"

namespace perception {

    bool Hear::update(const sexp_t* sexp) {
        if (!parser::SexpParser::parseValue(sexp, mTime)) {
            std::cerr << "[Hear Perception] can not get time" << std::endl;
            return false;
        }

        sexp = sexp->next;
        std::string direction;
        if (!parser::SexpParser::parseValue(sexp, direction)) {
            std::cerr << "[Hear Perception] can not get direction" << std::endl;
            return false;
        }

        if ("self" == direction) {
            mSelf = true;
        } else {
            mSelf = false;
            if (!parser::SexpParser::parseValue(sexp, mDirection)) {
                std::cerr << "[Hear Perception] can not parse the direction" << std::endl;
                return false;
            }
        }

        sexp = sexp->next;
        if (!parser::SexpParser::parseValue(sexp, mMsg)) {
            std::cerr << "[Hear Perception] can not get the message" << std::endl;
            return false;
        }
        if (mMsg[0] != (WM.getOurTeamIndex() == serversetting::TI_LEFT ? 'L' : 'R')) {
            //             std::cerr<<"[Hear Perception] this is not our message"<<std::endl;
            return false;
        }
        if (mMsg[1] != 'S') {
            return false;
        }
        //   std::cout <<"hera/////////////////////////////////////////////////"<<mMsg<<std::endl;

        // std::cout<<"nowTime"<< WM.getGameTime() <<" hear message : "<<mTime<<' '<<mSelf<<' '<<mDirection<<' '<<mMsg<<std::endl;

        //        std::stringstream ss;
        //        ss<<mMsg.substr(1, mMsg.find(",") );
        //        int time = 0;
        //        ss>>time;
        //        //////////////////Allen modify///////////////////////////
        //        int nowTi =  (int)(mTime*100);
        //        int t =nowTi - time;
        //
        //        //////////////////Allen modify///////////////////////////
        //        //cout <<time <<" "<<nowTi << " "<< mTime<< " " << t << "^^^^^^^^^^^^^^^^^";
        //        if ( t < 0 ){
        //            std::cerr<<"[Hear Perception] this message comes from future"<<std::endl;
        //            return false;
        //        }
        //        t = t%100;
        //        if ( t > 12 ){
        //            std::cerr<<"nowTime"<< WM.getGameTime() <<" hear message : "<<mTime<<' '<<mSelf<<' '<<mDirection<<' '<<mMsg<<std::endl;
        //            std::cerr<<"[Hear Perception] this message is too old!!!and t is"<< t<<std::endl;
        //            return false;
        //        }

        return true;
    }

} // namespace perception


