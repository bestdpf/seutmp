
#include "Time.h"
#include "parser/SexpParser.hpp"

namespace perception {

Time::Time(const sexp_t* sexp)
{
   if ( !update(sexp) )
   {
       mNow = 0;
       mStep = 0;
   }
}

bool Time::update(const sexp_t* sexp)
{
	//if ( !parser::SexpParser::parseGivenValue(sexp,"now",mNow) ) return false;
	//sexp = sexp->next;
	//if ( !parser::SexpParser::parseGivenValue(sexp,"step",mStep) ) return false;
	if ( !parser::SexpParser::parseValue(sexp->list->next,mNow) ) return false;
	/** there is no `step' now */
	//sexp = sexp->next;
	//if ( !parser::SexpParser::parseValue(sexp->list->next,mStep) ) return false;
	
	return true;
}

std::ostream& operator<<(std::ostream &stream, const Time& t)
{
    //stream<<"(time (now "<<t.mNow<<")(step "<<t.mStep<<"))";
	stream<<"(time (now "<<t.mNow<<"))";
    return stream;
}

} // namespace perception
