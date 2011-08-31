/*****************************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 * This file is orginal from rcssserver3D
 *
 * $Id: Vector.hpp,v 1.1.1.1 2007/03/07 03:28:10 xy Exp $
 *
 ****************************************************************************************/

#ifndef MATH_VECTOR_H
#define MATH_VECTOR_H

#include "TVector.hpp"

namespace math
{

    /**
     * convert the polar to the Decare position
     *
     * @param[in] pol the Vector3f of the polar
     * which stands for ( distance, theta, phi )
     *
     * @return Decare position
     */
    template <typename DATATYPE>
    TVector<DATATYPE,3> pol2xyz(const TVector<DATATYPE,3>& v)
    {
        DATATYPE cz = cosDeg(v.z());
        return TVector<DATATYPE,3>
            (
                v.x() * cosDeg(v.y()) * cz,

                v.x() * sinDeg(v.y()) * cz,

                v.x() * sinDeg(v.z())
                );
    }

    /** 
     * covert an 2D vector in Polar coordiantion to Decare coordination
     *  
     * @param v the 2D vector in Polar coordiantion
     * 
     * @return the 2D vector in Decare coordination
     */
    template <typename DATATYPE>
    TVector<DATATYPE,2> pol2xyz(const TVector<DATATYPE,2>& v)
    {
        return TVector<DATATYPE,2>
            (
                v.x() * cosDeg(v.y()),

                v.x() * sinDeg(v.y())
                );
    }

    /** 
     * covert an 2D vector in Decare coordiantion to Polar coordination
     *  
     * @param v the 2D vector in Decare coordiantion
     * 
     * @return the 2D vector in Polar coordination
     */
    template <typename DATATYPE>
    TVector<DATATYPE,2> xyz2pol(const TVector<DATATYPE,2>& v)
    {
        return TVector<DATATYPE,2>
            (
                v.length(),

                v.angle()
                );
    }

    /** return the two vector's clip angle
     *  @note only consider the 2D
     *  @param two vector
     *  @return the clip angle
     */
    template<typename DATATYPE, size_t ELEMENTS>
    AngDeg calClipAng(const TVector<DATATYPE,ELEMENTS>& v1,
                      const TVector<DATATYPE,ELEMENTS>& v2)
    {
        AngDeg ang1 = v1.angle();
        AngDeg ang2 = v2.angle();
        return normalizeAngle( ang1 - ang2 );
    }

    /** return the 3 points clip angle, the 3 points are:
     *  @note only consider the 2D
     *  p0------->p1
     *    \
     *     \
     *      >p2
     * @param 3 points
     * @return the clip angle
     */
    template<typename DATATYPE, size_t ELEMENTS>
    AngDeg calClipAng(const TVector<DATATYPE,ELEMENTS> &p0,
                      const TVector<DATATYPE,ELEMENTS> &p1,
                      const TVector<DATATYPE,ELEMENTS> &p2)
    {
        return calClipAng( p1-p0, p2-p0);
    }

    typedef TVector<int,2> Vector2i;
    
    typedef TVector<float,2> Vector2f;

    typedef TVector<float,3> Vector3f;
	
    typedef TVector<float,6> Vector6f;

} //namespace math

#endif //MATH_VECTOR_H
