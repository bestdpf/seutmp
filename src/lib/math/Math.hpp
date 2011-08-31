/*****************************************************************************************
 *                                      SEU-3D
 *                     -------------------------------------------------
 * Copyright (c) 2005, Yuan XU<xychn15@yahoo.com.cn>,Chang'e SHI<evelinesce@yahoo.com.cn>
 * Copyright (c) 2006, Yuan XU<xuyuan.cn@gmail.com>,Chunlu JIANG<JamAceWatermelon@gmail.com>
 * Southeast University ,China
 * All rights reserved.
 *
 * $Id: Math.hpp,v 1.1.1.1 2007/03/07 03:28:10 xy Exp $
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
 
 
#ifndef MATHS_HPP
#define MATHS_HPP

#ifdef __cplusplus
extern "C"
{
#endif



#ifdef __cplusplus
}
#endif

/** header from STL */
#include <algorithm>
#include <deque>
#include <map>
#include <string>
#include <set>
#include <vector>
#include <math.h>

#include "Defines.hpp"
#include "Angle.hpp"
#include "Number.hpp"
#include "Vector.hpp"
#include "Matrix.hpp"
#include "TConvexPolygon.hpp"

namespace math
{
	
//typedef float Time;		// Type definition for Time in 10ms
//typedef float Second;	// Type definition for Time in sencond
typedef unsigned short int Step; // Type definition for simulate step num
typedef unsigned short int Num; // Type definition for palyer's number.

    typedef TLine2<float> Line2f;
    
    typedef TSegment2<float> Segment2f;
    
    typedef TConvexPolygon<float> ConvexPolygonf;
    
} // namespace math

#endif /* MATHS_HPP */
