/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Matrix.hpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#ifndef MATH_MATRIX_HPP
#define MATH_MATRIX_HPP

#include "TMatrix.hpp"
#include "Angle.hpp"

namespace math {

template<typename DATATYPE>
class TransMatrix
{
public:
  /**
   * comment by dpf
   * the trans matrix here is not similar with the common one, it is inv of common one
   * so the left-product and right-product is revised compared to commone situation
   * */
	//
	// @note the 4*4 matrix is row order, that is
	//  | n_x, n_y, n_z, 0 | // x-axis right
	//  | o_x, o_y, o_z, 0 | // y-axis forward
	//  | a_x, a_y, a_z, 0 | // z-axis
	//  | p_x, p_y, p_z, 1 | // position
	// there are some function for spatial computation
	// the TransMatrix should satisfy these equations:
	// (1)  | n | = | o | = | a | = 1
	// (2)  n x o = a, o x a = n, a x n = o
	//      || n_x, n_y, n_z || 
	// (3)  || o_x, o_y, o_z || = 1
	//      || a_x, a_y, a_z ||
	//

    TransMatrix(){}
		
    TransMatrix(const TMatrix<DATATYPE,4,4>& rm)
    {
        set(rm(0,0), rm(0,1), rm(0,2),
              rm(1,0), rm(1,1), rm(1,2),
              rm(2,0), rm(2,1), rm(2,2),
              rm(3,0), rm(3,1), rm(3,2) );
    }

    TransMatrix(DATATYPE nx, DATATYPE ny, DATATYPE nz,
                DATATYPE ox, DATATYPE oy, DATATYPE oz,
                DATATYPE ax, DATATYPE ay, DATATYPE az,
                DATATYPE px, DATATYPE py, DATATYPE pz)
    {
        set(nx, ny, nz,
              ox, oy, oz,
              ax, ay, az,
              px, py, pz);
    }

    void set(DATATYPE nx, DATATYPE ny, DATATYPE nz,
                DATATYPE ox, DATATYPE oy, DATATYPE oz,
                DATATYPE ax, DATATYPE ay, DATATYPE az,
                DATATYPE px, DATATYPE py, DATATYPE pz)
    {
        mR[0].set(nx,ny,nz);
        mR[1].set(ox,oy,oz);
        mR[2].set(ax,ay,az);
        mP.set(px,py,pz);
    }

    void set( const TMatrix<DATATYPE,3,3>& rotate, const TVector<DATATYPE,3>& pos )
    {
        mR = rotate;
        mP = pos;
    }

    TransMatrix& identity()
    {
        mR.identity();
        mP.zero();
        return *this;
    }
  /**
   * comment by dpf
   * the trans matirx is inv of a common one 
   * */
	// sets up a X-rotation matrix with inAngle degrees
	f_inline void rotationX(AngDeg inAngle)
    {
        DATATYPE c=cosDeg(inAngle), s=sinDeg(inAngle);

        set(  1, 0, 0,
                0, c, s,
                0,-s, c,
                0, 0, 0 );
    }

    // sets up a Y-rotation matrix with inAngle degrees
    f_inline void rotationY(AngDeg inAngle)
    {
        DATATYPE c=cosDeg(inAngle), s=sinDeg(inAngle);

        set(  c, 0,-s,
                0, 1, 0,
                s, 0, c,
                0, 0, 0  );
    }

    // sets up a Z-rotation matrix with inAngle degrees
    f_inline void rotationZ(AngDeg inAngle)
    {
        DATATYPE c=cosDeg(inAngle), s=sinDeg(inAngle);

        set(  c, s, 0,
               -s, c, 0,
                0, 0, 1,
                0, 0, 0 );
    }

    f_inline TransMatrix& operator-=(const TransMatrix& T)
    {
        mR -= T.R();
        mP -= T.p();
        return *this;
    }
    f_inline TransMatrix& operator=(const TransMatrix & T)
    {
       mR=T.R();
       mP=T.p();
       return *this;
    }
    f_inline TransMatrix& operator*=(const TransMatrix& T)
    {
        mR *= T.R();
        mP = T.transform(mP);
        return *this;
    }

    f_inline TransMatrix operator*(const TransMatrix& T) const
    {
        TransMatrix r(*this);
        return r *= T;
    }

    f_inline TransMatrix operator-(const TransMatrix& T) const
    {
        TransMatrix r(*this);
        return r -= T;
    }
	
    /** transform by a TransMatrix */
	/** 
     * calculate the global matrix of T, which is the local matrix in
     * this coordinate
     * *this is global to rel trans matrix; comment by dpf
     * @param T the local matrix
     * 
     * @return the global matrix
     * comment by dpf
     * this func in fact is *this = T*(*this) is *this right-product T
     * but in common situation, it should be left-product in global transfer
     */
    f_inline TransMatrix& transfer(const TransMatrix& T)
    {
		transfer(T.p());
        return transfer(T.R());
    }
	

    /** 
     * calculate the local matrix of T, while it is in the same
     * coordinate (global) as this matrix
     * 
     * @param T the global matrix
     * 
     * @return the local matrix in this coordinate that equals the T
     * in global
     */
    f_inline TransMatrix& inverseTransfer(const TransMatrix& T)
    {
		inverseTransfer(T.p());
        return inverseTransfer(T.R());
    }

    /** 
     * apply only rotation to the matrix
     * 
     * @param R the rotation
     * 
     * @return the totated matrix
     */
    f_inline TransMatrix& transfer(const TMatrix<DATATYPE,3,3>& R)
    {
        mR = R * mR;
        return *this;
    }

    /** 
     * rotate the matrix alone a given line
     * 
     * @param DATATYPE the data type
     * @param v the vector of the line
     * @param ang the rotated angle
     * 
     * @return 
     */
    f_inline TransMatrix& transfer(const TVector<DATATYPE,3>& v, AngDeg ang)
    {
        TMatrix<DATATYPE,3,3> R = rodrigues(v,ang);
        return transfer(R);
    }
	
    f_inline TransMatrix& inverseTransfer(const TMatrix<DATATYPE,3,3>& R)
    {
        mR = R * (~mR);
        return *this;
    }
	
    /** transfer by a position */
    f_inline TransMatrix& transfer(const TVector<DATATYPE,3> & p)
    {
        mP = transform(p);
        return *this;
    }
	
	f_inline TransMatrix& inverseTransfer(const TVector<DATATYPE,3> & p)
    {
        mP = inverseTransform(p);
        return *this;
    }
    
    f_inline TransMatrix& rotateLocalX(AngDeg inAngle)
    {
        TransMatrix T;
        T.rotationX(inAngle);
        return transfer(T.R());
    }

    f_inline TransMatrix& rotateLocalY(AngDeg inAngle)
    {
        TransMatrix T;
        T.rotationY(inAngle);
        return transfer(T.R());
    }

    f_inline TransMatrix& rotateLocalZ(AngDeg inAngle)
    {
        TransMatrix T;
        T.rotationZ(inAngle);
        return transfer(T.R());
    }
	
	f_inline TransMatrix& rotateGlobalX(AngDeg inAngle)
    {
        TransMatrix T;
        T.rotationX(inAngle);
        mR *= T.R();
		return *this;
    }

    f_inline TransMatrix& rotateGlobalY(AngDeg inAngle)
    {
        TransMatrix T;
        T.rotationY(inAngle);
        mR *= T.R();
        return *this;
    }

    f_inline TransMatrix& rotateGlobalZ(AngDeg inAngle)
    {
        TransMatrix T;
        T.rotationZ(inAngle);
        mR *= T.R();
        return *this;
    }
	
	
	
    // this only applies the rotation part of the matrix
    f_inline TVector<DATATYPE,3> rotate(const TVector<DATATYPE,3>  & inVector) const
    {
        DATATYPE x = inVector.x(), y = inVector.y(), z = inVector.z();
        return TVector<DATATYPE,3>
                                (x*n().x() + y*o().x() + z*a().x(),
                                x*n().y() + y*o().y() + z*a().y(),
                                x*n().z() + y*o().z() + z*a().z());
    }
/**
 * comment by dpf
 * *this is a transfer matrix from A to B, and inVector is a vector in A, the 
 * result is the value of inVector in coordinate system B.
 * like global to rel transfer
 **/
    f_inline TVector<DATATYPE,3> transform(const TVector<DATATYPE,3>& inVector) const
    {
		TVector<DATATYPE,3> res = rotate(inVector);
		res += p();
		return res;
	}
	
	f_inline TVector<DATATYPE,3> inverseTransform(const TVector<DATATYPE,3>& inVector) const
    {
		TVector<DATATYPE,3> temp = inVector;
		temp -= p();
		TVector<DATATYPE,3> res = inverseRotate(temp);
		return res;
	}
	 
	// this applies the transpose of the rotation part of the matrix (inverse rotation)
	f_inline TVector<DATATYPE,3> inverseRotate(const TVector<DATATYPE,3>& inVector) const
	{
		return TVector<DATATYPE,3>(inVector.dot(n()), inVector.dot(o()), inVector.dot(a()) );
	}
	
	//  | n_x, n_y, n_z, 0 | // x-axis right
	//  | o_x, o_y, o_z, 0 | // y-axis forward
	//  | a_x, a_y, a_z, 0 | // z-axis up
	//  | p_x, p_y, p_z, 1 | // position
    f_inline TMatrix<DATATYPE,3,3>& R() { return mR; }
    f_inline const TMatrix<DATATYPE,3,3>& R() const { return mR; }

    f_inline TVector<DATATYPE,3>& n() { return mR[0]; }
    f_inline TVector<DATATYPE,3>& o() { return mR[1]; }
    f_inline TVector<DATATYPE,3>& a() { return mR[2]; }
    f_inline TVector<DATATYPE,3>& p() { return mP; }

    f_inline const TVector<DATATYPE,3>& n() const { return mR[0]; }
    f_inline const TVector<DATATYPE,3>& o() const { return mR[1]; }
    f_inline const TVector<DATATYPE,3>& a() const { return mR[2]; }
    f_inline const TVector<DATATYPE,3>& p() const { return mP; }

    f_inline TVector<DATATYPE,3>& right() { return mR[0]; }
    f_inline TVector<DATATYPE,3>& forward() { return mR[1]; }
    f_inline TVector<DATATYPE,3>& up() { return mR[2]; }
    f_inline TVector<DATATYPE,3>& pos() { return mP; }

    f_inline const TVector<DATATYPE,3>& right() const { return mR[0]; }
    f_inline const TVector<DATATYPE,3>& forward() const { return mR[1]; }
    f_inline const TVector<DATATYPE,3>& up() const { return mR[2]; }
    f_inline const TVector<DATATYPE,3>& pos() const { return mP; }

    f_inline TVector<DATATYPE,3>& operator[](size_t i)
    {
        if (i>2) return mP;
        return mR[i];
    }

    f_inline const TVector<DATATYPE,3>& operator[](size_t i) const
    {
        if (i>2) return mP;
        return mR[i];
    }

    f_inline DATATYPE operator()(size_t i, size_t j) const
    {
        if ( j == 3 )
        {
            if ( i == 3 ) return DATATYPE(1);
            return DATATYPE(0);
        }
        return (*this)[i][j];
    }

	//           | n_x,  o_x,  a_x,  0 |
	//  T^(-1) = | n_y,  o_y,  a_y,  0 |
	//           | n_z,  o_z,  a_z,  0 |
	//           | -p*n, -p*o, -p*a, 1 |
    f_inline TransMatrix operator!() const
    {
        TransMatrix T(*this);
        return T.inv();
    }

    f_inline TransMatrix& inv()
    {
        mR.transpose();
        mP = -rotate(mP);
        return *this;
    }

    // calculate the rotated angle of the matrix
    // accodring to rotation-Z, rotation-Y, rotation-X order
    /* | cz*cy          sz*cy          -sy   0 |
       | cz*sy*sx-sz*cx sz*sy*sx+cz*cx cy*sx 0 |
       | cz*sy*cx+sz*sx sz*sy*cx-cz*sx cy*cx 0 |
       | 0              0              0     1 | */
    DATATYPE rotatedAngX() const
    {
        return atan2Deg(mR(1,2), mR(2,2));
    }

    DATATYPE rotatedAngY() const
    {
        return asinDeg(-mR(0,2));
    }

    DATATYPE rotatedAngZ() const
    {
        return atan2Deg(mR(0,1),mR(0,0));
    }

    DATATYPE squareLength() const
        {
            return mR.squareLength() + mP.squareLength();
        }

    DATATYPE length() const
        {
            return sqrt(squareLength());
        }
    
private:
    TMatrix<DATATYPE,3,3> mR;
    TVector<DATATYPE,3> mP;
};

typedef TMatrix<float,1,2> Matrix1x2f;
typedef TMatrix<float,2,1> Matrix2x1f;
typedef TMatrix<float,1,3> Matrix1x3f;
typedef TMatrix<float,3,1> Matrix3x1f;


typedef TMatrix<float,2,2> Matrix2x2f;
typedef TMatrix<float,3,3> Matrix3x3f;
typedef TMatrix<float,4,4> Matrix4x4f;

typedef TMatrix<float,6,1> Matrix6x1f;
typedef TMatrix<float,6,6> Matrix6x6f;
typedef TMatrix<float,3,6> Matrix3x6f;

typedef TransMatrix<float> TransMatrixf;

template<typename DATATYPE>
TMatrix<DATATYPE,3,3> rodrigues(TVector<DATATYPE,3> w, AngDeg ang)
{
	TMatrix<DATATYPE,3,3> e;
	e.identity();
    DATATYPE l = w.length();
    if ( l < EPSILON ) // zero
    {
        return e;
    }

    w /= l; // normalize the axis
	TMatrix<DATATYPE,3,3> a(TVector<DATATYPE,3>(0, w[2], -w[1]),
							TVector<DATATYPE,3>(-w[2], 0, w[0]),
							TVector<DATATYPE,3>(w[1], -w[0], 0));
	e = e + a*sinDeg(ang) + a*a*(1-cosDeg(ang));
	return e;
}

///    w = (lnR)v
    template<typename DATATYPE>
    TVector<DATATYPE,3> rot2omega(const TMatrix<DATATYPE,3,3>& R)
    {
        DATATYPE theta = acos( (R(0,0)+R(1,1)+R(2,2)-1)/2 );
        DATATYPE k = theta/2/sin(theta);
        if ( isnan(k) ){
            k = 0.5;
        }
        return TVector<DATATYPE,3>(R(1,2)-R(2,1),
                                       R(2,0)-R(0,2),
                                       R(0,1)-R(1,0))*k;
    }

// dump
template <typename DATATYPE>
std::ostream& operator<<(std::ostream& os, const math::TransMatrix<DATATYPE>& m)
{
    os<<m.n()<<" 0\n"<<m.o()<<" 0\n"<<m.a()<<" 0\n"<<m.p()<<" 1";
    return os;
}

} // namespace math

#endif // MATH_MATRIX_HPP
