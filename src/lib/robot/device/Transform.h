/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_TRANSFORM_H_
#define _ROBOT_DEVICE_TRANSFORM_H_

#include "Device.h"

namespace robot {
    namespace device {

        class Transform : public Device
        {
        public:
            Transform();
        
            virtual ~Transform();

            void setLocalPos(const Vector3f& pos);

            /** 
             * @NOTE: this function is set, not rotate!
             * 
             * @param rot rotation angel (in degree) of X,Y,Z
             */
            void setLocalRotation(const Vector3f& rot);

            const TransMatrixf& getLocalMat() const
                {
                    return mLocalMat;
                }

            const TransMatrixf& getGlobalMat() const
                {
                    return mGlobalMat;
                }

            void setGlobalMat(const TransMatrixf& m )
                {
                    mGlobalMat = m;
                }

        private:
            /** the position&rotation in global */
            TransMatrixf mGlobalMat;
            
            /** the position&rotation in local */ 
            TransMatrixf mLocalMat;
        };
    
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_TRANSFORM_H_ */
