/* 
 * File:   SegmentPol.cpp
 * Author: robocup
 * 
 * Created on 2011年3月30日, 下午2:06
 */

#include "SegmentPol.h"

namespace perception {

//    SegmentPol::SegmentPol() {
//    };

//    SegmentPol::~SegmentPol() {
//    };

    bool SegmentPol::update(const sexp_t* sexp) {
        const sexp_t* t = sexp;
       const std::string nan = "nan";
         std::string  value = "";
         //   const sexp_t* t = sexp->list;


//          parser::SexpParser::parseGivenValue(t, "pol", value);
//                std::cout << "value"<<value << std::endl;
//            if(nan==value){
//                std::cout <<  "value"<<value <<std::endl;
//            }

     //   std::cout<<"seeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"<<std::endl;
        for (int i = 0; i < 2;i++ ) {
          //  std::cout<<"in"<<mP[i]<<std::endl;
         parser::SexpParser::parseGivenValue(t, "pol", value);
          if(nan==value){
                std::cerr <<  "value"<<value <<std::endl;
                return false;
            }
            if (!parser::SexpParser::parseGivenValue(t, "pol", mP[i])) {
                // mPosMap[oid] = calLocalRelPos(mPolMap[oid]);
                std::cout << "errrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr" << std::endl;
                return false;
            }
          //  std::cout<<"out"<<mP[i]<<std::endl;
            t = t->next;
        }
        return true;
    }

    std::ostream & operator<<(std::ostream &stream, const SegmentPol& l) {
        stream << "(L ";
        //	for( GyroRate::TRateMap::const_iterator iter = g.mRates.begin(); iter!=g.mRates.end(); ++iter )
        //	{
        stream << "(pol " << l.mP[0] << ")(" << l.mP[1] << ")";
        //	}
        stream << ")";
        return stream;
    }
}
