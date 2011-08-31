/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef PERCEPTION_CONSTANT_KALMAN_FILTER_HPP
#define PERCEPTION_CONSTANT_KALMAN_FILTER_HPP

namespace perception 
{

/** @class ConstantKalmanFilter
	@brief a simple implemention of Kalman filter to estimate the random constant
	
	The Process Model:
		x[k] = x[k-1} + w[k-1]

	The Filter Equations and Paramters:
		x'[k] = x[k-1]
		P'[k] = P[k-1]

		K[k] = P'[k] / ( P'[k]+P[k] )
		x[k] = x'[k] + K[k]*( z[k] - x'[k] )
		P[k] = ( 1-K[k] )*P'[k]
	more information see http://www.cs.unc.edu/~welch/kalman/
*/
template<class ValueT, class VariantT>
class ConstantKalmanFilter
{
 public:
    ConstantKalmanFilter()
        {
            reset();
        }

    ~ConstantKalmanFilter()
        {
        }
    
    void reset()
        {
            mReset = true;
        }

    void addInput( ValueT value, VariantT var )
        {
            if ( mReset )// first value, intialize
            {
                mTotalVar  = var;            
                mWeightedValue = value;
                mReset = false;
            }
            else
            {
                // otherwise use new value based on weighted variance kalman filtering technique
                VariantT k = mTotalVar / ( var + mTotalVar );
                mWeightedValue += k*( value - mWeightedValue );
                mTotalVar -= k*mTotalVar;
            }
        }

    bool getOutput( VariantT& weightedValue, VariantT& totalVar ) const
        {
            if ( mReset ) return false;
		
            weightedValue = mWeightedValue;
            totalVar = mTotalVar;
            return true;
        }

    bool getValue( VariantT& weightedValue) const
        {
            if ( mReset ) return false;
            weightedValue = mWeightedValue;
            return true;
        }
    
    
 private:
    
    VariantT mWeightedValue;
    
    VariantT mTotalVar;
    
    bool mReset;
};
 
    
} // namspace perception


#endif // PERCEPTION_CONSTANT_KALMAN_FILTER_HPP
