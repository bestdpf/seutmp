/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: ClassException.hpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/
 
/** @file ClassException.hpp
 *  help to create a exception class of the CLASS
 */

#ifndef CLASS_EXCEPTION_HPP
#define CLASS_EXCEPTION_HPP

#include <cstring>
#include <string>
#include <errno.h>

/**
 *   Signals a problem with the execution of a CLASS call.
 */
template<class CLASS>
class ClassException
{
public:
    /**
    *   Construct a Exception with a explanatory message.
    *   @param message explanatory message
    *   @param incSysMsg true if system message (from strerror(errno))
    *   should be postfixed to the user provided message
    */
    ClassException(const std::string &message, bool inclSysMsg = false) throw()
    : mMessage(message)
    {
        if (inclSysMsg)
        {
            mMessage.append(": ");
            mMessage.append(strerror(errno));
        }
    }
	
	/**  Provided just to guarantee that no exceptions are thrown. */
    ~ClassException() throw() {}

    /** Get the exception message
    *   @return exception message
    */
    const char *what() const throw()
    {
        return mMessage.c_str();
    }

private:
	std::string mMessage;  // Exception message
};

#endif //CLASS_EXCEPTION_HPP
