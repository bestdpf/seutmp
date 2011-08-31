/*******************************************************************************************
 *                                      SEU-3D
 *                     -------------------------------------------------
 * Copyright (c) 2005, Yuan XU<xychn15@yahoo.com.cn>,Chang'e SHI<evelinesce@yahoo.com.cn>
 * Copyright (c) 2006, Yuan XU<xuyuan.cn@gmail.com>,Chunlu JIANG<JamAceWatermelon@gmail.com>
 * Southeast University ,China
 * All rights reserved.
 *
 * $Id: Angle.hpp,v 1.1.1.1 2007/03/07 03:28:10 xy Exp $
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
 * @file Angle.h
 *
 * auxiliary goniometric functions which enable you to
 * specify angles in degrees rather than in radians
 */
 
#ifndef ANGLE_HPP
#define ANGLE_HPP

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

typedef float AngRad;  // Type definition for angles in degrees.
typedef float AngDeg;  // Type definition for angles in radians.
	

/** This function converts an angle in radians to the corresponding angle in
 *  degrees.
 *  @param[in] x an angle in radians
 *  @return the corresponding angle in degrees
 */
template<class TYPE>
TYPE rad2Deg( TYPE x )
{
  return ( x * 180.0f / M_PI );
}

/** This function converts an angle in degrees to the corresponding angle in
 *  radians.
 *  @param x an angle in degrees
 *  @return the corresponding angle in radians
 */
template<class TYPE>
TYPE deg2Rad( TYPE x )
{
  return ( x * M_PI / 180.0f );
}

/** This function returns the cosine of a given angle in degrees using the
 *  built-in cosine function that works with angles in radians.
 *  @param x an angle in degrees
 *  @return the cosine of the given angle
 */
template<class TYPE>
TYPE cosDeg( TYPE x )
{
  return ( cos( deg2Rad( x ) ) );
}

/** This function returns the sine of a given angle in degrees using the
 *  built-in sine function that works with angles in radians.
 *  @param x an angle in degrees
 *  @return the sine of the given angle
 */
template<class TYPE>
TYPE sinDeg( TYPE x )
{
  return ( sin( deg2Rad( x ) ) );
}

/** This function returns the tangent of a given angle in degrees using the
 *  built-in tangent function that works with angles in radians.
 *  @param x an angle in degrees
 *  @return the tangent of the given angle
 */
template<class TYPE>
TYPE tanDeg( TYPE x )
{
  return ( tan( deg2Rad( x ) ) );
}

/** This function returns the principal value of the arc tangent of x
 *  in degrees using the built-in arc tangent function which returns
 *  this value in radians.
 *  @param[in] x a double value
 *  @return the arc tangent of the given value in degrees
 */
template<class TYPE>
TYPE atanDeg( TYPE x )
{
  return ( rad2Deg( atan( x ) ) );
}

/** This function returns the principal value of the arc tangent of y/x in
 *  degrees using the signs of both arguments to determine the quadrant of the
 *  return value. For this the built-in 'atan2' function is used which returns
 *  this value in radians.
 *  @param[in] x a value
 *  @param[in] y a value
 *  @return the arc tangent of *y/x* in degrees taking the signs of x and y into
 *  account
 */
template<class TYPE>
TYPE atan2Deg( TYPE y, TYPE x )
{
  if( fabs( y ) < EPSILON && fabs( x ) < EPSILON )
    return ( 0.0 );

  return ( rad2Deg( atan2( y, x ) ) );
}

/** This function returns the principal value of the arc cosine of x in degrees
 *  using the built-in arc cosine function which returns this value in radians.
 *  @param[in] x a value
 *  @return the arc cosine of the given value in degrees
 */
template<class TYPE>
TYPE acosDeg( TYPE x )
{
  if( x >= 1 )
    return ( 0.0 );
  else if( x <= -1 )
    return ( 180.0 );

  return ( rad2Deg( acos( x ) ) );
}

/** This function returns the principal value of the arc sine of x in degrees
 *  using the built-in arc sine function which returns this value in radians.
 *  @param x a value
 *  @return the arc sine of the given value in degrees
 */
template<class TYPE>
TYPE asinDeg( TYPE x )
{
  if( x >= 1 )
    return ( 90.0 );
  else if ( x <= -1 )
    return ( -90.0 );

  return ( rad2Deg( asin( x ) ) );
}

/** This function returns a boolean value which indicates whether the value
 * 'ang' (from interval [-180..180] lies in the interval [angMin..angMax].
 *  Examples: isAngInInterval( -100, 4, -150) returns false
 *           isAngInInterval(   45, 4, -150) returns true
 *  @param[in] ang angle that should be checked
 *  @param[in] angMin minimum angle in interval
 *  @param[in] angMax maximum angle in interval
 *  @return boolean indicating whether ang lies in [angMin..angMax]
 */
template<class TYPE>
bool isAngInInterval( TYPE ang, TYPE angMin, TYPE angMax )
{
  // convert all angles to interval 0..360
  if( ( ang    + 360 ) < 360 ) ang    += 360;
  if( ( angMin + 360 ) < 360 ) angMin += 360;
  if( ( angMax + 360 ) < 360 ) angMax += 360;

  if( angMin < angMax ) // 0 ---false-- angMin ---true-----angMax---false--360
    return angMin < ang && ang < angMax ;
  else                  // 0 ---true--- angMax ---false----angMin---true---360
    return !( angMax < ang && ang < angMin );
}

/** This method returns the bisector (average) of two angles. It deals
 *  with the boundary problem, thus when 'angMin' equals 170 and 'angMax'
 *  equals -100, -145 is returned.
 *  @param[in] angMin minimum angle [-180,180]
 *  @param[in] angMax maximum angle [-180,180]
 *  @return average of angMin and angMax.
 */
template<class TYPE>
TYPE calBisectorTwoAngles( TYPE angMin, TYPE angMax )
{
  // separate sine and cosine part to circumvent boundary problem
  /*return normalizeAngle(
            atan2Deg( (sinDeg( angMin) + sinDeg( angMax ) )/2.0,
                      (cosDeg( angMin) + cosDeg( angMax ) )/2.0 ) );*/
    TYPE ang = (angMin+angMax)*TYPE(0.5);
    if ( angMin > angMax ) ang += TYPE(180);
    return normalizeAngle( ang );
}

/** 濮瑰倷琚辨稉顏囶潡鎼达妇娈戞径纭咁潡 calculate the clip angle of two angles
 * @note +180閸?180鎼达妇娈戞潏鍦櫕闂傤噣顣?閹碘偓娴犮儰绗夐懗鐣屾纯閹恒儱鍣? * @param[in] ang1 the begin angle
 * @param[in] ang2 the end angle
 * @return the clip ang of the two angles
 */
template<class TYPE>
TYPE calClipAng( TYPE ang1, TYPE ang2 )
{
	return normalizeAngle(ang1-ang2);
}

/** This method normalizes an angle. This means that the resulting
 *  angle lies between -180 and 180 degrees.
 *  @param[in] angle the angle which must be normalized
 *  @return the result of normalizing the given angle
 */
template<class TYPE>
TYPE normalizeAngle( TYPE angle )
{
	while( angle > 180.0  ) angle -= 360.0;
	while( angle < -180.0 ) angle += 360.0;

	return ( angle );
}

} // namespace math

#endif /* ANGLE_HPP */
