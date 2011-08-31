/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

/**
 * @file   TSegment.hpp
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Sat Oct 13 03:46:50 2007
 * 
 * @brief  2D segment template class
 * 
 * 
 */

#ifndef MATH_TSEGMENT2_HPP
#define MATH_TSEGMENT2_HPP

#include "TLine2.hpp"

namespace math{

    template <typename DATATYPE>
    class TSegment2: public TLine2<DATATYPE>
    {
        /// endpoints
        TVector<DATATYPE,2> mP[2];
    public:
        TSegment2()
            {
            }

        TSegment2( const TVector<DATATYPE,2>& p0, const TVector<DATATYPE,2>& p1)
            :TLine2<DATATYPE>(p0,p1)
            {
                mP[0] = p0;
                mP[1] = p1;
            }

        const TVector<DATATYPE,2>& p0() const
            {
                return mP[0];
            }

        const TVector<DATATYPE,2>& p1() const
            {
                return mP[1];
            }

        bool isBetween(const TVector<DATATYPE,2>& p) const
            {
                return p[0]<=std::max(mP[0][0],mP[1][0])
                    && p[0]>=std::min(mP[0][0],mP[1][0])
                    && p[1]<=std::max(mP[0][1],mP[1][1])
                    && p[1]>=std::min(mP[0][1],mP[1][1]);
            }

        DATATYPE length() const
            {
                return ( mP[0]-mP[1] ).length();
            }

        bool isContain(const TVector<DATATYPE,2>& p) const
            {
                return (p-mP[0]).length() + (p-mP[1]).length()
                    < length() + EPSILON;                
            }
        
        /** 
         * calculate the intersection of the segment and a given line
         * 
         * @param line the given line
         * @param intersection retrun the intersection
         * 
         * @return if the segment and line have intersection
         */
        bool calIntersection(const TLine2<DATATYPE>& line,
                             TVector<DATATYPE,2>& intersection)const
            {
                intersection = TLine2<DATATYPE>::calIntersection(line);
                return isBetween(intersection);
            }

        /** 
         * calculate the interseciton of two segment
         * 
         * @param seg2 the other segment
         * @param intersection return the intersection
         * 
         * @return if the tow segments have intersection
         */
        bool calIntersection(const TSegment2& seg2,
                             TVector<DATATYPE,2>& intersection)const
            {
                if ( calIntersection(TLine2<DATATYPE>(seg2), intersection ) ){
                    return seg2.isContain(intersection);
                }
                return false;
            }

        /** 
         * if there is an intersection of two segment
         * 
         * @param seg2 the other segment
         * 
         * @return if the tow segments have intersection
         */
        bool isIntersect(const TSegment2& seg2) const
            {
                TVector<DATATYPE,2> intersection;
                return calIntersection(seg2, intersection);
            }
    };

} // namespace math

#endif // MATH_TSEGMENT2_HPP


       
