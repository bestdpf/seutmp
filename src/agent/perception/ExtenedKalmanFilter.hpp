/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef _EXTENEDKALMANFILTER_H_
#define _EXTENEDKALMANFILTER_H_

#include "math/Matrix.hpp"

/** @class ExtenedKalmanFilter
	@brief an implemention of extended kalman filter
	
	The Process Model:
    x[k] = f(x[k-1], u[k-1], w[k-1])
    z[k] = h(x[k], v[k])

	The Filter Equations and Paramters:
    x'[k] = f(x[k-1], u[k-1], 0)
    P'[k] = A*P[k-1]* A^T + W[k]*Q[k-1]*W[k]^T

    K[k] = P'[k]* H^T / ( H*P'[k]* H^T + V[k]*R*V[k]^T )
    x[k] = x'[k] + K[k]*( z[k]-h(x'[k],0) )
    P[k] = ( I - K[k]*H ) * P'[k]
*/
template <class ValueT>
class ExtenedKalmanFilter
{
public:
	/*===========================*/
	/* 公有函数/Public Functions  */
	/*===========================*/

	/** 缺省构造函数/default Constructor */
	ExtenedKalmanFilter(){};
		
	/** 析构函数/Destructors */
	~ExtenedKalmanFilter(){};
	
	/** 初始化 */
	void init(const math::TMatrix<ValueT,2,1>& x,
              const math::TMatrix<ValueT,2,2>& A,
              const math::TMatrix<ValueT,2,2>& B,
              const math::TMatrix<ValueT,2,2>& H,
              const math::TMatrix<ValueT,2,2>& W,
              const math::TMatrix<ValueT,2,2>& V,
              const math::TMatrix<ValueT,2,2>& Q,
              const math::TMatrix<ValueT,2,2>& R)
              {
                  mx = x;
                  mA = A;
                  mB = B;
                  mH = H;
                  mW = W;
                  mV = V;
                  mQ = Q;
                  mR = R;
                  mP.zero();
                  mI.identity();
              }

              void predict(const math::TMatrix<ValueT,2,1>& x)
              {
                  mx = x;
		
                  // P'[k] = A*P[k-1]* A^T + W[k]*Q[k-1]*W[k]^T
                  mP = mA*mP*mA.transpose() + mW*mQ*mW.transpose();

                  // GL_PRINT("PVEKF","PVEKF A: %f %f %f %f",_A(0,0),_A(0,1),_A(1,0),_A(1,1));
                  // GL_PRINT("PVEKF","PVEKF W: %f %f %f %f",_W(0,0),_W(0,1),_W(1,0),_W(1,1));
                  // GL_PRINT("PVEKF","PVEKF Q: %f %f %f %f",_Q(0,0),_Q(0,1),_Q(1,0),_Q(1,1));
                  // GL_PRINT("PVEKF","PVEKF P': %functions %f %f %f",_P(0,0),_P(0,1),_P(1,0),_P(1,1));   
              }
	
              void observe(const math::TMatrix<ValueT,2,1>& z)
              {
                  mz = z;
              }
	
              void correct()
              {
                  // K[k] = P'[k]* H^T / ( H*P'[k]* H^T + V[k]*R*V[k]^T )
                  math::TMatrix<ValueT,2,2> K = mP*mH.transpose()
                      * (mH*mP*mH.transpose() + mV*mR*mV.transpose()).inv();
		
                  // x[k] = x'[k] + K[k]*( z[k]-h(x'[k],0) )
                  mx = mx + K*(mz-mx);
		
                  // P[k] = ( I - K[k]*H ) * P'[k]
                  mP = ( mI - K*mH ) * mP;
		
                  // GL_PRINT("PVEKF","PVEKF V: %functions %f %f %f",_V(0,0),_V(0,1),_V(1,0),_V(1,1));
                  // GL_PRINT("PVEKF","PVEKF K: %f %f %f %f",K(0,0),K(03,1),K(1,0),K(1,1));
                  // GL_PRINT("PVEKF","PVEKF P: %f %f %f %f",_P(0,0),_P(0,1),_P(1,0),_P(1,1));
              }
	
    void setP(ValueT v0, ValueT v1, ValueT v2, ValueT v3)
        {
            mP(0,0) = v0;
            mP(0,1) = v1;
            mP(1,0) = v2;
            mP(1,1) = v3;
        }
	
    const math::TMatrix<ValueT,2,1>& getOutput() const { return mx; }
	
    const math::TMatrix<ValueT,2,1>& update(const math::TMatrix<ValueT,2,1>& x,
                                            const math::TMatrix<ValueT,2,1>& z)
        {
            predict(x);
            observe(z);
            correct();
            return getOutput();
        }
    
protected:
    /*===========================*/
    /* 私有成员/Private Attributes */
    /*===========================*/
    math::TMatrix<ValueT,2,1> mx;
    math::TMatrix<ValueT,2,1> mz;
    math::TMatrix<ValueT,2,2> mA, mB, mH, mW, mV, mQ, mR, mP;
    math::TMatrix<ValueT,2,2> mI;
};

/** @class PVExtenedKalmanFilter
	@brief an implemention of extended kalman filter to estimate position and velocity

	and in particular,
    x[0] = velocity
    x[1] = position
	so,
    A[0][1] = B[0][1] = B[1][0] = B[1][1] = u[1] = 0
    A[1][1] = 1
    A[1][0] = t
    A[0][0] = 1 - mu*t/m
    B[0][0] = t/m
    W[0][0] = u[0]
    W[1][0] = u[0]*t
    W[0][1] = W[1][1] = 0
    V[1][1] = ...// something like dist from flag
    V[0][0] = V[1][1]/t
    V[1][0] = V[0][1] = 0
    H[0][0] = H[1][1] = 1
    H[0][1] = H[1][0] = 0
    Q[0][0] = the error factor of predict
    Q[0][1] = Q[1][0] = Q[1][1] = 0
    R[0][0] = R[1][0] = R[0][1] = 0
    R[1][1] = the error factor of observe
*/
    template<class T>
    class PVExtenedKalmanFilter: public ExtenedKalmanFilter<T>
    {
    public:
        /*===========================*/
        /* 公有函数/Public Functions  */
        /*===========================*/

        /** 缺省构造函数/default Constructor */
        PVExtenedKalmanFilter(){};
		
        /** 析构函数/Destructors */
        ~PVExtenedKalmanFilter(){};
	
        /** 初始化 */
        void init(T x0, T x1, T t, T mu, T m, T errPredict, T errObserve)
            {
                math::TMatrix<T,2,1> x;
                x(0,0) = x0;
                x(0,1) = x1;
                T A[2][2] = {{1-mu*t/m,0},{t,1}};
                T B[2][2] = {{t/m,0},{0,0}};
                T H[2][2] = {{1,0},{0,1}};
                T W[2][2] = {{0,0},{0,0}}; // W[0][0] = u[0]
                T V[2][2] = {{0,0},{0,0}}; // V[1][1] = ...
                T Q[2][2] = {{errPredict,0},{0,errPredict}};
                T R[2][2] = {{errObserve,0},{0,errObserve}};
                
                ExtenedKalmanFilter<T>::init(
                    x,
                    math::TMatrix<T,2,2>(A),
                    math::TMatrix<T,2,2>(B),
                    math::TMatrix<T,2,2>(H),
                    math::TMatrix<T,2,2>(W),
                    math::TMatrix<T,2,2>(V), 
                    math::TMatrix<T,2,2>(Q),
                    math::TMatrix<T,2,2>(R));
            }
	
        void update(T& vel, T& pos, T f, T t,T oVel, T oPos, T oVar)
            {
                f = fabs(f);
                // GL_PRINT("PVEKF","f=%f oErr=%f",f,oVar);
                this->mW(0,0) = f;
                this->mW(0,1) = 0.f;
                this->mW(1,0) = 0.f;
                this->mW(1,1) = 0.f;
                this->mV(0,0) = oVar/t;
                this->mV(0,1) = 0;
                this->mV(1,0) = 0;
                this->mV(1,1) = oVar;
                math::TMatrix<T,2,1> pre;
                pre(0,0) = vel;
                pre(1,0) = pos;
                math::TMatrix<T,2,1> obs;
                obs(0,0) = oVel;
                obs(1,0) = oPos;
                math::TMatrix<T,2,1> res = ExtenedKalmanFilter<T>::update(pre, obs);
                vel = res(0,0);
                pos = res(1,0);
            }
    };

#endif /* _EXTENEDKALMANFILTER_H_ */
