
#include "Joint.h"
#include "parser/SexpParser.hpp"

namespace perception {

    using namespace math;

    Joint::Joint(const sexp_t* sexp)
    {
        if ( !update(sexp) )
        {
            mAngle = 0;
        }
        mRate = 0;
    }
    
    Joint::Joint(const sexp_t* sexpAx, const sexp_t* sexpRt)
    {
        if ( !update(sexpAx, sexpRt) )
        {
            mAngle = 0;
            mRate = 0;
        }
    }

    bool Joint::update(const sexp_t* sexpAx, const sexp_t* sexpRt)
    {
        //(angle 0.000733871) (rate 1.58129e-10)
        /*
          if ( !parser::SexpParser::parseGivenValue(sexp,"ax",mAngle)
          && !parser::SexpParser::parseGivenValue(sexp,"ax1",mAngle)
          && !parser::SexpParser::parseGivenValue(sexp,"ax2",mAngle) )
          return false;
    
          sexp = sexp->next;
          if ( !parser::SexpParser::parseGivenValue(sexp,"rt",mRate)
          && !parser::SexpParser::parseGivenValue(sexp,"rt1",mRate)
          && !parser::SexpParser::parseGivenValue(sexp,"rt2",mRate) )
          return false;
        */
        if ( !parser::SexpParser::parseValue(sexpAx->list->next,mAngle) )
            return false;
    
        //  there is no `rate' now, try it for debugging
        if ( 0 != sexpRt ){
            parser::SexpParser::parseValue(sexpRt->list->next,mRate);
        }
	
        return true;
    }

    bool Joint::update(const sexp_t* sexp)
    {
        //(angle 0.000733871)
        /*
          if ( !parser::SexpParser::parseGivenValue(sexp,"ax",mAngle)
          && !parser::SexpParser::parseGivenValue(sexp,"ax1",mAngle)
          && !parser::SexpParser::parseGivenValue(sexp,"ax2",mAngle) )
          return false;
        */
        return parser::SexpParser::parseValue(sexp->list->next,mAngle);
    }

Joint& Joint::operator+=(AngDeg ang)
{
	mAngle += ang;
	mAngle = normalizeAngle(mAngle);
	return *this;
}

std::ostream& operator<<(std::ostream &stream, const Joint& j)
{
    //stream<<"(angle "<<j.mAngle<<")(rate "<<j.mRate<<")";
	stream<<"[ax "<<j.mAngle<<"]";
    return stream;
}

} // namespace perception
