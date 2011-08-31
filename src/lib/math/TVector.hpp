/*****************************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 * This file is orginal from rcssserver3D
 *
 * $Id: TVector.hpp,v 1.2 2007/08/23 13:53:05 chen Exp $
 *
 ****************************************************************************************/
#ifndef MATH_TVECTOR_H
#define MATH_TVECTOR_H

#include "Defines.hpp"
#include "Angle.hpp"
#include <iostream>

namespace math
{

/**  TVector is a template class for vector implementations. It
 *  abstracts away the number of elements and their type.
 */
template <typename DATATYPE, size_t ELEMENTS>
class TVector
{
    //
    // functions
    //
public:
    TVector() {}

    // Copy constructor:
    TVector(const TVector &v)
    {
        for (size_t i=0; i<ELEMENTS; i++)
            mEle[i] = v[i];
    }

    /** 2D vector functions */
    /** constructs a Vector from x and y */
    TVector(DATATYPE x, DATATYPE y)
    { set(x, y); }

    /** sets up the vector from x and y */
    f_inline const TVector<DATATYPE,2>& set(const DATATYPE& x, const DATATYPE& y)
    {
        this->mEle[0] = x;
        this->mEle[1] = y;
        return *this;
    }

    /** 3D vector functions */
    /** constructs a vector from x,y and z */
    TVector(const DATATYPE& x, const DATATYPE& y, const DATATYPE& z)
    { set(x, y, z); }

    /** sets up the vector from x,y and z */
    const TVector<DATATYPE,3>& set(const DATATYPE& x, const DATATYPE& y, const DATATYPE& z)
    {
        this->mEle[0] = x;
        this->mEle[1] = y;
        this->mEle[2] = z;
        return *this;
    }

    /** calculates the cross product, returning a new TVector3 */
    const TVector<DATATYPE,3> cross(const TVector<DATATYPE,3>& v) const
    {
        return TVector<DATATYPE,3>(
                    mEle[1] * v[2] - mEle[2] * v[1],
                    mEle[2] * v[0] - mEle[0] * v[2],
                    mEle[0] * v[1] - mEle[1] * v[0] );
    }

    // accessors

    /** returns a reference to a row of the vector */
    f_inline DATATYPE& operator[](size_t row)
    { return mEle[row]; }

    /** returns a constant reference to a row of a vector */
    f_inline const DATATYPE& operator[](size_t row) const
    { return mEle[row]; }

     // Element Access operators

    /** returns a reference to the first component */
    f_inline DATATYPE& x()
    { return this->mEle[0]; }

    /** returns a constant reference to the first component */
    f_inline const DATATYPE& x() const
    { return this->mEle[0]; }

    /** return value of the first component */
    f_inline DATATYPE getX()
    { return this->mEle[0]; }

    /** set value of the first component */
    f_inline void setX(DATATYPE v)
    { mEle[0] = v; }

    /** returns a reference to the second component */
    f_inline DATATYPE& y()
    { return this->mEle[1]; }

    /** returns a constant reference to the second component */
    f_inline const DATATYPE& y() const
    { return this->mEle[1]; }

    /** return value of the second component */
    f_inline DATATYPE getY()
    { return this->mEle[1]; }

    /** set value of the second component */
    f_inline void setY(DATATYPE v)
    { mEle[1] = v; }
    
    /** returns a reference to the third component */
    f_inline DATATYPE& z()
    { return this->mEle[2]; }

    /** returns a constant reference to the third component */
    f_inline const DATATYPE& z() const
    { return this->mEle[2]; }

    /** return value of the third component */
    f_inline DATATYPE getZ()
    { return this->mEle[2]; }

    /** set value of the third component */
    f_inline void setZ(DATATYPE v)
    { mEle[2] = v; }
    
    /** copies another vector 'copy' */
    f_inline const TVector& set(const DATATYPE *copy)
    {
        for (size_t i=0; i<ELEMENTS; i++)
            mEle[i] = copy[i];
        return *this;
    }

    /** return a pointer to the encapsulated vector */
    f_inline DATATYPE* get()
    { return mEle; }

    f_inline const DATATYPE* get() const
    { return mEle; }

    /** fills all components of the vector with value 'v' */
    f_inline TVector& fill(const DATATYPE& v)
    {
        for (size_t c=0; c < ELEMENTS; c++)
            mEle[c] = v;
        return *this;
    }

    /** sets all components of the vector to 0 */
    f_inline TVector& zero()
    {  return fill(0);  }

    f_inline bool isNan() const
        {
            for ( size_t c=0; c<ELEMENTS; c++ )
            {
                if ( isnan(mEle[c]) ) return true;
            }
            return false;
		}
    
    // operators
	template <typename DATATYPE_R, size_t ELEMENTS_R>
	f_inline TVector& operator=(const TVector<DATATYPE_R,ELEMENTS_R>& v)
	{
		for ( size_t c=0; c<ELEMENTS; c++ )
		{
			if ( c < ELEMENTS_R ) mEle[c] = v[c];
			else mEle[c] = 0;
		}
		return *this;
	}
	
    /** calculates this VECTOR + VECTOR */
    f_inline const TVector operator+(const TVector &v) const
    {
        TVector r(*this);
        return r+=v;
    }

    /** calculates VECTOR - VECTOR*/
    f_inline const TVector operator-(const TVector &v) const
    {
        TVector r(*this);
        return r-=v;
    }

    /** calculates VECTOR * DATATYPE */
    f_inline const TVector operator*(const DATATYPE &v) const
    {
        TVector r(*this);
        return r*=v;
    }

    /** calculates VECTOR / DATATYPE */
    f_inline const TVector operator/(const DATATYPE &v) const
    {
        TVector r(*this);
        return r/=v;
    }

    /** add another vector */
    f_inline TVector& operator+=(const TVector &v)
    {
        for (size_t c=0; c < ELEMENTS; c++)
            mEle[c] += v[c];
        return *this;
    }

    /** substracts another vector */
    f_inline TVector& operator-=(const TVector &v)
    {
        for (size_t c=0; c < ELEMENTS; c++)
            mEle[c] -= v[c];
        return *this;
    }

    /** scale */
    f_inline TVector& operator*=(const DATATYPE &v)
    {
         for (size_t c=0; c < ELEMENTS; c++)
            mEle[c] *= v;
        return *this;
    }

    /** divides */
    f_inline TVector& operator/=(const DATATYPE &v)
    {
        DATATYPE c = DATATYPE(1)/v;
        return (*this)*=c;
    }

    /** returns the negate of this vector */
    f_inline TVector operator-() const
    {
        TVector r;
        for (size_t c=0; c < ELEMENTS; c++)
            r[c] = -mEle[c];
        return r;
    }

    /** returns true if this vector and v are equal */
    f_inline bool operator==(const TVector& v)const
    {
        return (0==memcmp(this,& v, sizeof(*this)));
    }

    /** returns true if this vector and v are not equal */
    f_inline bool operator!=(const TVector& v)const
    {
        return !(*this==v);
    }

    /** returns the dot product from this vector and v */
    f_inline DATATYPE dot(const TVector& v) const
    {
        DATATYPE r = mEle[0] * v[0];
        for (size_t c=1; c < ELEMENTS; c++)
            r += mEle[c] * v[c];
        return r;
    }

    /** normalizes the vector */
    f_inline const TVector& normalize()
    {
        DATATYPE len = length();
        return (*this)/=len;
    }

    /** calculates the normalized vector, not modifying the vector */
    f_inline TVector normalized() const
    {
        TVector r(*this);
        return r.normalize();
    }

    /** calculates the squared length of the vector */
    f_inline DATATYPE squareLength() const
    {
        DATATYPE r = mEle[0] * mEle[0];
        for (size_t c=1; c<ELEMENTS; c++)
            r += mEle[c] * mEle[c];
        return r;
    }

    /** calculates the length of the vector */
    f_inline DATATYPE length() const
    { return sqrt(squareLength()); }

    f_inline DATATYPE angle() const
        {
            return atan2Deg(mEle[1],mEle[0]);
        }
	
	/** whether this point is on the line determined by p1 and p2 */
	f_inline bool isOnLine( const TVector& p1, const TVector& p2, const float epsilon = 0.1f ) const
	{
     return (p1 - *this).isParallelWith( (p2 - p1), epsilon);
	}
	
	/** whether this vector parallels l  */
	f_inline bool isParallelWith( const TVector& l, const float epsilon = 0.1f ) const 
	{
     DATATYPE lambada = 0;
     for( size_t c = 1; c < ELEMENTS; c ++ )
     {
       if( abs(mEle[c] * l.mEle[c]) <= (epsilon*epsilon) && (abs(mEle[c]) + abs(l.mEle[c])) > (epsilon+epsilon) )
       {
         return false;
       }
       else
			{
           if( 0 == lambada ) 
           {
             if( abs(mEle[c]) <= epsilon && abs(l.mEle[c]) <= epsilon )
               continue;
             else
               lambada = mEle[c] / l.mEle[c];
           }
           else if( abs( (float) (mEle[c] - l.mEle[c] * lambada) ) <= epsilon )
             continue;
           else 
             return false;
			}			
     }
		
     return true;
	} 
  
protected:
    DATATYPE mEle[ELEMENTS];
};

template <typename DATATYPE, size_t ELEMENTS>
std::ostream& operator <<(std::ostream& ost, const TVector<DATATYPE,ELEMENTS>& v)
{
    if (ELEMENTS < 1) return ost;
    ost << v[0];
    for (size_t i=1; i<ELEMENTS; ++i) ost << " " << v[i];
    return ost;
}

// DATATYPE * vector
template <typename DATATYPE, size_t ELEMENTS>
f_inline const TVector<DATATYPE, ELEMENTS> operator*(const DATATYPE& f, const TVector<DATATYPE, ELEMENTS>& vec)
{
    return vec * f;
}

} //namespace math

#endif //MATH_TVECTOR_H
