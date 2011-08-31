
#ifndef PERCEPTION_TIME_H
#define PERCEPTION_TIME_H

#include "math/Math.hpp"
#include "BasicPerception.h"
#include <iostream>

namespace perception {
/*
(time 
 (now 7.71999)
*/
class Time: public BasicPerception
{
public:
    Time():mNow(0),mStep(0.02){};
    
	Time(const sexp_t* sexp);
	
	~Time(){};
	
	virtual bool update(const sexp_t* sexp);
	
	friend std::ostream& operator<<(std::ostream &stream, const Time& t);
		
	float now() const { return mNow; }

    /** 
     * get the reference of mNow
     * 
     * @return the reference of mNow
     */
    float& now() { return mNow; }

    float step() const { return mStep; }

    float& step() 
        {
            return mStep;
        }
	
private:
	float mNow, mStep;
};

std::ostream& operator<<(std::ostream &stream, const Time& t);

} // namespace perception

#endif // PERCEPTION_TIME_H
