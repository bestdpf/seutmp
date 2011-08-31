/*****************************************************************************************
 *                                      SEU-3D
 *                     -------------------------------------------------
 * Copyright (c) 2005, Yuan XU<xychn15@yahoo.com.cn>,Chang'e SHI<evelinesce@yahoo.com.cn>
 * Copyright (c) 2006, Yuan XU<xuyuan.cn@gmail.com>,Chunlu JIANG<JamAceWatermelon@gmail.com>
 * Southeast University ,China
 * All rights reserved.
 *
 * $Id: Number.hpp,v 1.1.1.1 2007/03/07 03:28:10 xy Exp $
 *
 * Additionally,this program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 ****************************************************************************************/

/**
 *  @file Number.hpp
 *
 * auxiliary numeric functions
 */
 
#ifndef NUMBER_H
#define NUMBER_H

#ifdef __cplusplus
extern "C"
{
#endif



#ifdef __cplusplus
}
#endif

#include "Defines.hpp"

namespace math
{

/** This function returns the sign of a give value.
 *  1 is positive, -1 is negative
 *  @param[in] d1 first parameter
 *  @return the sign of this value */
template<class T>
int sign( T d1 )
{
  return (d1>0)?1:(d1<0?-1:0);
}

/** generate a random value in (min,max)
 * @param[in] min the min value of the random number
 * @param[in] max the max value of the random number
 * @return the random number
 */
template < class T >
T random( T min, T max )
{
	static bool haveSet = false;
	if(!haveSet)
	{
		srand(time(0));
		haveSet = true;
	}
	return (T(rand())/RAND_MAX)*(max-min)+min;
}

/** set the value to range [min,max]
 *  @param[in] x the orginal value
 *  @param[in] min the range left
 *  @param[in] max the range right
 *  @return the value be clamped
 */
template < class T >
T clamp( T x, T min, T max )
{
	if ( x > max ) return max;
	if ( x < min ) return min;
	return x;
}

/** x^2
 *  @param[in] x
 *  @return x^2
 */
template < class T >
T pow2( T x )
{
	return x*x;
}

template < class T >
bool inRange( T v, T minV, T maxV )
{
    return v > minV && v < maxV;
}

}// namespace math

#endif /* NUMBER_H */
