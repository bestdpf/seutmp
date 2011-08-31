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
 * @file   TLine2.hpp
 * @author Xu Yuan <xuyuan.cn@gmail.com.cn>
 * @date   Sat Oct 13 02:49:46 2007
 * 
 * @brief  the 2D line template class
 * 
 * 
 */


#ifndef MATH_TLINE2_HPP
#define MATH_TLINE2_HPP

#include "Number.hpp"
#include "TVector.hpp"
#include "Angle.hpp"

namespace math{

    template <typename DATATYPE>
    class TLine2
    {
        // a line is defined by the formula: ay + bx + c = 0
        DATATYPE mA; 
        DATATYPE mB; 
        DATATYPE mC; 
    public:

        /** 
         * create line by given coefficients
         * 
         * @param a the a coefficients
         * @param b the b coefficients
         * @param c the c coefficients
         */
        TLine2(DATATYPE a=1, DATATYPE b=0, DATATYPE c=-1)
            :mA(a),mB(b),mC(c){}
        
         /** 
          * create line from tow points
          * 
          * @param pos1 the one point
          * @param pos2 the other point
          */
         TLine2( const TVector<DATATYPE,2> &pos1,
                 const TVector<DATATYPE,2> &pos2)
            {
                // 1*y + bx + c = 0 => y = -bx - c
                // with -b the direction coefficient (or slope)
                // and c = - y - bx
                DATATYPE dTemp = pos2[0] - pos1[0]; // determine the slope
                if( fabs(dTemp) < EPSILON ){
                    // ay + bx + c = 0 with vertical slope=> a = 0, b = 1
                    mA = 0.0;
                    mB = 1.0;
                }
                else{
                    // y = (-b)x -c with -b the slope of the line
                    mA = 1.0;
                    mB = -(pos2[1] - pos1[1])/dTemp;
                }
                // ay + bx + c = 0 ==> c = -a*y - b*x
                mC =  - mA*pos2.y()  - mB * pos2.x();
            }

        /** 
         * create line from one point and an angle
         * 
         * @param pos the point in line
         * @param angle the sclope angle
         */
        TLine2( const TVector<DATATYPE,2> &pos, AngDeg angle )
            {
                // calculate point somewhat further in direction 'angle'
                // and make line from these two points.
                TVector<DATATYPE,2> pos2 = pos;
                pos2.x() += cosDeg(angle);
                pos2.y() += sinDeg(angle);
                *this = TLine2( pos, pos2);
            }
        
        DATATYPE a() const { return mA;}
        DATATYPE b() const { return mB;}
        DATATYPE c() const { return mC;}

        DATATYPE& a() { return mA;}
        DATATYPE& b() { return mB;}
        DATATYPE& c() { return mC;}


        /** 
         * calculate the intersection of two lines
         * 
         * @param line the other line
         * 
         * @return the intersection
         */
        TVector<DATATYPE,2> calIntersection( const TLine2 &line ) const
            {
                DATATYPE x, y;
                if( ( mA / mB ) ==  (line.a() / line.b() )){
                    // lines are parallel, no intersection
                    return TVector<DATATYPE,2>(0,0);
                }

                if( mA == 0 ) {
                    // bx + c = 0 and a2*y + b2*x + c2 = 0 ==> x = -c/b
                    // calculate x using the current line
                    x = -mC/mB;// and calculate the y using the second line
                    y = line.calYGivenX(x);
                }
                else if( line.a() == 0 ){
                    // ay + bx + c = 0 and b2*x + c2 = 0 ==> x = -c2/b2
                    x = -line.c()/line.b();
                    // calculate x using
                    y = calYGivenX(x);
                    // 2nd line and calculate y using current line
                }
                    // ay + bx + c = 0 and a2y + b2*x + c2 = 0
                    // y = (-b2/a2)x - c2/a2
                    // bx = -a*y - c =>  bx = -a*(-b2/a2)x -a*(-c2/a2) - c ==>
                    // ==> a2*bx = a*b2*x + a*c2 - a2*c
                    // ==> x = (a*c2 - a2*c)/(a2*b - a*b2)
                    // calculate x using the above formula
                    // and the y using the current line
                else{
                    x = (mA*line.c() - line.a()*mC)/(line.a()*mB - mA*line.b());
                    y = calYGivenX(x);
                }
                return TVector<DATATYPE,2>( x, y );
            }

        /** 
         * ay + bx + c = 0 -> y = (-b/a)x + (-c/a)
         * tangent: y = (a/b)*x + C1 -> by - ax + C2 = 0 => C2 = ax - by
         * with pos.y = y, pos.x = x
         * 
         * @param pos the tangent intersection
         * 
         * @return the tangent line of this line
         */
        TLine2 calTangentLine( const TVector<DATATYPE,2> &pos ) const
            {
                return TLine2( mB, -mA, mA*pos.x() - mB*pos.y() );
            }

        /** 
         * calculate the closest point of a given point to this line
         * 
         * @param pos the given point
         * 
         * @return the closest point
         */
        TVector<DATATYPE,2>
        calPointOnLineClosestTo( const TVector<DATATYPE,2> &pos ) const
            {
                TLine2 l2 = calTangentLine( pos );  // get tangent line
                // and intersection between the two lines
                return calIntersection( l2 );
            }

        /** 
         * calculate the distance from a given point to this line
         * 
         * @param pos the given point
         * 
         * @return the distance from the pos to this line
         */
        DATATYPE calDistanceToPoint( const TVector<DATATYPE,2> &pos ) const
            {
                return (calPointOnLineClosestTo( pos )-pos).length();
            }

        /** 
         * calculate the Y value by given X value
         * 
         * @param x the given x coordinate value
         * 
         * @return the Y coordinate value
         */
        DATATYPE calYGivenX( DATATYPE x ) const
            {
                if( mA == 0 )
                {
                    std::cerr << __FILE__<<__LINE__<<__FUNCTION__
                              <<" Cannot calculate Y coordinate: "<< std::endl;
                    return 0;
                }
                // ay + bx + c = 0 ==> y = -(b*x + c)/a
                return -(mB*x+mC)/mA;
            }

        /** 
         * calculate the X value by given Y value
         * 
         * @param y the given Y coordinate value
         * 
         * @return the X coordinate value
         */
        DATATYPE calXGivenY( DATATYPE y ) const
            {
                if( mB == 0 )
                {
                    std::cerr << __FILE__<<__LINE__<<__FUNCTION__
                              <<"Cannot calculate X coordinate\n"<<std::endl;
                    return 0;
                }
                // ay + bx + c = 0 ==> x = -(a*y + c)/b
                return -(mA*y+mC)/mB;
            }

        /** 
         * calculate the location of point to the line
         * 
         * @param p the given point
         * 
         * @return +1 means upper the line
         *          0 means on the line
         *         -1 means lower the line
         */
        int location(const TVector<DATATYPE,2>& p)const
            {
                return sign(mA*p.y()+mB*p.x()+mC);
            }
			
		/**
		* calculate the ang for line
		*/			
		AngDeg getAngle()
			{
				if(mA < EPSILON )
					return 90.0f;
				return 	atanDeg(-mB/mA);
			}

        /** 
         * @param[in] pos1 first point
 	     * @param pos2 second point
         * @return 
         */
        template<typename P_T>
        static TLine2 makeMidperpendicularFromTwoPoints(const P_T &pos1, const P_T &pos2 )
 	        {
                TLine2 l1(pos1,pos2);
                P_T posMid = (pos1+pos2)*0.5;
                return l1.calTangentLine(posMid);
            }
        
    };

    /// dump
    template<typename DATATYPE>
    std::ostream& operator <<(std::ostream & os, TLine2<DATATYPE> l)
    {
        DATATYPE a = l.a();
        DATATYPE b = l.b();
        DATATYPE c = l.c();
        // ay + bx + c = 0 -> y = -b/a x - c/a
        if( a == 0 ){
            os << "x = " << -c/b;
        }
        else{
            os << "y = ";
            if( b != 0 )
                os << -b/a << "x ";
            if( c > 0 )
                os << "- " <<  fabs(c/a);
            else if( c < 0 )
                os << "+ " <<  fabs(c/a);
        }
        return os;
    }
} // namespace math
        
#endif // MATH_TLINE2_HPP
