/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: TMatrix.hpp,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#ifndef MATH_TMATRIX_HPP
#define MATH_TMATRIX_HPP

#include "Vector.hpp"
#include "../ClassException.hpp"

namespace math {

/**  TMatrix is a template class for matrix implementations. It
 *  abstracts away the number of row, column and their type.
 */
#define MATRIX_EXCEPTION 0
#if MATRIX_EXCEPTION
#define  THROW_MATRIX_EXECPTION throw(ClassException<TMatrix>)
#define ASSERT_MATRIX_EXCEPTION(c,m) if(!c){  throw ClassException<TMatrix>(m);}
#else
#define  THROW_MATRIX_EXECPTION
#define ASSERT_MATRIX_EXCEPTION(c,m)
#endif

template <typename DATATYPE, size_t ROW, size_t COLUMN>
class TMatrix
{
public:
    f_inline TMatrix(){}

    f_inline TMatrix(const TMatrix& rm)
    {
        *this = rm;
    }

    f_inline TMatrix(const DATATYPE v[ROW][COLUMN])
    {
        *this = v;
    }

    f_inline TMatrix( const TVector<DATATYPE,COLUMN>& r0,
                              const TVector<DATATYPE,COLUMN>& r1,
                              const TVector<DATATYPE,COLUMN>& r2 )
    {
        set( r0, r1, r2 );
    }

    f_inline TMatrix<DATATYPE,3,COLUMN>& set( const TVector<DATATYPE,COLUMN>& r0,
                                       const TVector<DATATYPE,COLUMN>& r1,
                                       const TVector<DATATYPE,COLUMN>& r2 )
    {
        mEle[0] = r0;
        mEle[1] = r1;
        mEle[2] = r2;
        return *this;
    }

    f_inline ~TMatrix()
    {}

    // assignment  
    f_inline TMatrix& operator = (const TMatrix& rm)
    {
        for ( size_t i=0; i<ROW; i++ )
                mEle[i] = rm[i];
        return *this;
    }

     f_inline TMatrix& operator = (const DATATYPE v[ROW][COLUMN])
    {
        for ( size_t i=0; i<ROW; i++ )
            for( size_t j=0; j<COLUMN; j++ )
                mEle[i][j] = v[i][j];
        return *this;
    }

    // returns a reference to a row of the matrix,
    // so that we can get the value by m[r][c]
    f_inline TVector<DATATYPE,COLUMN>& operator[](size_t row) THROW_MATRIX_EXECPTION
    {
        ASSERT_MATRIX_EXCEPTION( (row < ROW), "operator[] Index out of range!");
        return mEle[row];
    }

    f_inline const TVector<DATATYPE,COLUMN>& operator[](size_t row) const  THROW_MATRIX_EXECPTION
    {
        ASSERT_MATRIX_EXCEPTION( (row < ROW), "operator[]const Index out of range!");
        return mEle[row];
    }

    // data accessor
    f_inline DATATYPE& operator()(size_t row, size_t col)  THROW_MATRIX_EXECPTION
    {
        ASSERT_MATRIX_EXCEPTION( (row < ROW && col < COLUMN) ,"operator() Index out of range!");
        return mEle[row][col];
    }

    f_inline DATATYPE operator()(size_t row, size_t col) const  THROW_MATRIX_EXECPTION
    {
        ASSERT_MATRIX_EXCEPTION( (row < ROW && col < COLUMN) ,"operator()const Index out of range!");
        return mEle[row][col];
    }

    // set all data to 0
    f_inline TMatrix& zero()
    {
        for (size_t i=0; i < ROW; i++)
            mEle[i].zero();
        return *this;
    }

    // set the identity matrix
    f_inline TMatrix<DATATYPE, COLUMN, ROW>& identity()
    {
        zero();
        for (size_t i=0; i < ROW; i++)
            mEle[i][i] = 1;
        return *this;
    }

    /** operators */
	f_inline TMatrix& operator+=(const TMatrix& rm)
	{
        for (size_t i=0; i < ROW; i++)
                mEle[i] += rm[i];
        return *this;
    }

    f_inline TMatrix& operator-=(const TMatrix& rm)
    {
        for (size_t i=0; i < ROW; i++)
                mEle[i] -= rm[i];
        return *this;
	}

    f_inline TMatrix& operator*=( DATATYPE scale)
    {
        for (size_t i=0; i < ROW; i++)
                mEle[i] *= scale;
        return *this;
    }

    f_inline TMatrix& operator*=(const TMatrix& rm)
    {
        *this = *this * rm;
        return *this;
    }

    f_inline TMatrix& operator /= ( DATATYPE c )
    {
        *this *= (DATATYPE(1)/c);
        return *this;
    }

    /** the pow of matrix:
     *  A^3 = A*A*A
    */
    f_inline TMatrix& operator ^= ( size_t exp )
    {
        TMatrix temp(*this);

        for (size_t i=2; i <= exp; i++)
            *this *= temp;

        return *this;
    }

    f_inline TMatrix operator+(const TMatrix& rm)const
    {
        TMatrix res(*this);
        res += rm;
        return res;
    }

    f_inline TMatrix operator-(const TMatrix& rm)const
    {
        TMatrix res(*this);
        res -= rm;
        return res;
    }

    f_inline TMatrix operator*(DATATYPE scale)const
    {
        TMatrix res(*this);
        res *= scale;
        return res;
    }
	
	f_inline TVector<DATATYPE,ROW> operator*(const TVector<DATATYPE,COLUMN>& v)const
	{
		TVector<DATATYPE,ROW> res;
		for(size_t i=0; i<ROW; i++)
		{
			res[i] = mEle[i].dot(v);
		}
		return res;
	}

    template <size_t COLUMN2>
    f_inline TMatrix<DATATYPE, ROW, COLUMN2> operator*(const TMatrix<DATATYPE, COLUMN, COLUMN2>& rm)const
    {
        TMatrix<DATATYPE, ROW, COLUMN2> res;
        for (size_t i=0; i < ROW; i++)
        {
            for( size_t j=0; j< COLUMN2; j++ )
            {
                res[i][j] = DATATYPE(0);
                for( size_t k=0; k < COLUMN; k++ )
                {
                    res[i][j]+=mEle[i][k] * rm(k,j);
                }
            }
        }
        return res;
    }

    f_inline TMatrix& operator / ( DATATYPE c ) const
    {
        return *this * (DATATYPE(1)/c);
    }

    f_inline TMatrix operator^( size_t exp )const
    {
        TMatrix res(*this);
        res ^= exp;
        return res;
    }

    // logical equal-to operator
    f_inline bool operator == ( const TMatrix& rm ) const
    {
        for (size_t i=0; i < ROW; i++)
                if ( mEle[i] != rm[i] )
                    return false;
        return true;
    }

    // logical no-equal-to operator
    f_inline bool operator != (const TMatrix& rm) const
    {
        return !( *this == rm );
    }

    f_inline TMatrix& transpose()
    {
        for (size_t i=0; i < ROW; i++)
            for( size_t j=i+1; j< COLUMN; j++ )
                std::swap( mEle[j][i], mEle[i][j] );
        return *this;
    }

    // inversion function
    f_inline TMatrix<DATATYPE, COLUMN, ROW>& inv () THROW_MATRIX_EXECPTION
    {
        size_t i,j,k;
        DATATYPE a1,a2;

        TMatrix<DATATYPE, COLUMN, ROW> temp;

        temp.identity();
        for (k=0; k < ROW; k++)
        {
            int indx = pivot(k);
            ASSERT_MATRIX_EXCEPTION( (indx != -1) , "TMatrix::operator!: Inversion of a singular matrix");
            if ( indx == -1 ) return *this; // can to get the result

            if (indx != 0)
            {
                std::swap( temp[k], temp[size_t(indx)] );
            }

            a1 = DATATYPE(1)/mEle[k][k];
            for (j=0; j < ROW; j++)
            {
                mEle[k][j] *= a1;
                temp(k,j) *= a1;
            }
            for (i=0; i < ROW; i++)
            {
                if (i != k)
                {
                    a2 = mEle[i][k];
                    for (j=0; j < ROW; j++)
                    {
                        mEle[i][j] -= a2 * mEle[k][j];
                        temp(i,j) -= a2 * temp(k,j);
                    }
                }
            }
        }
        *this = temp;
        return *this;
    }

    // solve simultaneous equation A*x = B --> x = A^(-1) * B
    template <size_t EQUS>
    f_inline TMatrix<DATATYPE,COLUMN, EQUS> solve (const TMatrix<DATATYPE, COLUMN, EQUS>& v) const THROW_MATRIX_EXECPTION
    {
        size_t i,j,k;
        DATATYPE a1;

        TMatrix<DATATYPE,ROW, COLUMN+EQUS> temp;
        TMatrix<DATATYPE,ROW, EQUS> s;
        for (i=0; i < ROW; i++)
        {
            for (j=0; j < COLUMN; j++)
                temp[i][j] = mEle[i][j];
            for (k=0; k < EQUS; k++)
                temp[i][COLUMN+k] = v(i,k);
        }
        for (k=0; k < ROW; k++)
        {
            int indx = temp.pivot(k);
            ASSERT_MATRIX_EXCEPTION( (indx != -1), "TMatrix::solve(): Singular matrix!");
            if ( indx == -1 ) return s; // can to get the result

            a1 = DATATYPE(1)/temp(k,k);
            for (j=k; j < COLUMN+EQUS; j++)
                temp[k][j] *= a1;

            for (i=k+1; i < ROW; i++)
            {
                a1 = temp(i,k);
                for (j=k; j < COLUMN+EQUS; j++)
                    temp[i][j] -= a1 * temp(k,j);
            }
        }

        for (k=0; k < EQUS; k++)
            for (int m=int(ROW)-1; m >= 0; m--)
            {
                s[m][k] = temp(m,COLUMN+k);
                for (j=m+1; j < COLUMN; j++)
                    s[m][k] -= temp(m,j) * s(j,k);
               }
        return s;
    }

     // partial pivoting method
    f_inline int pivot (size_t row)
    {
        size_t k = row;
        DATATYPE amax,temp;

        amax = 0.0;
        for (size_t i=row; i < ROW; i++)
            if ( (temp = fabs( mEle[i][row] )) > amax )
            {
                amax = temp;
                k = i;
            }
        if (mEle[k][row] == DATATYPE(0))
            return -1;
        if ( k != row )
        {
            std::swap(mEle[k],mEle[row]);
            return k;
        }
        return 0;
    }

    // calculate the determinant of a matrix
    f_inline DATATYPE det () const
    {
        size_t i,j,k;
        DATATYPE piv,detVal = DATATYPE(1);

        TMatrix<DATATYPE,COLUMN,ROW> temp(*this);

        for (k=0; k < ROW; k++)
        {
            int indx = temp.pivot(k);
            if (indx == -1)
                return 0;
            if (indx != 0)
                detVal = - detVal;
            detVal = detVal * temp(k,k);
            DATATYPE c = DATATYPE(1)/ temp(k,k);
            for (i=k+1; i < ROW; i++)
            {
                piv = temp(i,k) * c;
                for (j=k+1; j < ROW; j++)
                    temp[i][j] -= piv * temp(k,j);
            }
        }
        return detVal;
	}
	
	f_inline DATATYPE squareLength() const
	{
		DATATYPE sl = 0;
		for( size_t i=0; i<ROW; i++ )
		{
			sl += mEle[i].squareLength();
		}
		return sl;
	}
	
	f_inline DATATYPE length() const
	{
		return sqrt(squareLength());
	}

protected:
    TVector<DATATYPE, COLUMN> mEle[ROW];
};

} // namespace math

/** inversed matrix */
template <typename DATATYPE, size_t ROW, size_t COLUMN>
f_inline math::TMatrix<DATATYPE, COLUMN, ROW> operator ! ( const math::TMatrix<DATATYPE, ROW, COLUMN>& m )
{
   math::TMatrix<DATATYPE, COLUMN, ROW> temp(m);
   return temp.inv();
}

/** tarnsposed matrix */
template <typename DATATYPE, size_t ROW, size_t COLUMN>
f_inline math::TMatrix<DATATYPE, ROW, COLUMN> operator ~ ( const math::TMatrix<DATATYPE, ROW, COLUMN>& m )
{
    math::TMatrix<DATATYPE, ROW, COLUMN> temp(m);
   return temp.transpose();
}

template <typename DATATYPE, size_t ROW, size_t COLUMN>
f_inline math::TMatrix<DATATYPE, ROW, COLUMN> operator*(DATATYPE scale, math::TMatrix<DATATYPE, ROW, COLUMN>& m)
{
    return (m*scale);
}

// input stream function
template <typename DATATYPE, size_t ROW, size_t COLUMN>
f_inline std::istream& operator >> (std::istream& is, math::TMatrix<DATATYPE, ROW, COLUMN>& m)
{
   for (size_t i=0; i < ROW; i++)
      for (size_t j=0; j < COLUMN; j++)
      {
         DATATYPE x;
         is >> x;
         m(i,j) = x;
      }
   return is;
}

// dump
template <typename DATATYPE, size_t ROW, size_t COLUMN>
f_inline std::ostream& operator<<(std::ostream& os, const math::TMatrix<DATATYPE, ROW, COLUMN>& m)
{
    for( size_t i=0; i< ROW; i++ )
    {
        for( size_t j=0; j<COLUMN; j++ )
        {
            os<<m(i,j)<<"\t";
        }
        os<<'\n';
    }
    return os;
}

#endif // MATH_TMATRIX_HPP
