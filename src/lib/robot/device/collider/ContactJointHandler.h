/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_COLLIDER_CONTACTJOINTHANDLER_H_
#define _ROBOT_DEVICE_COLLIDER_CONTACTJOINTHANDLER_H_

#include "CollisionHandler.h"


namespace robot {
    namespace device {
        namespace collider {

            class ContactJointHandler : public CollisionHandler
            {
            public:

                ContactJointHandler();
                
                void setContactBounceMode(bool set);

                void setContactSlipMode(bool set);

                void setContactSoftERPMode(bool set);

                void setContactSoftCFMMode(bool set);

                void setContactBounceValue(float bounce);

                void setMinBounceVel(float vel);
                
                void setContactSlip(float slip);

                void setContactSoftERP(float erp);

                void setContactSoftCFM(float cfm);

            private:
                bool mBounceMode;
                bool mSlipMode;
                bool mSoftERPMode;
                bool mSoftCFMMode;
                float mBounceValue;
                float mMinBounceVel;
                float mSlip;
                float mSoftERP;
                float mSoftCFM;
            };
    
        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */


#endif /* _ROBOT_DEVICE_COLLIDER_CONTACTJOINTHANDLER_H_ */
