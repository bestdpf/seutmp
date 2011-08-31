/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "ContactJointHandler.h"

namespace robot {
    namespace device {
        namespace collider {

            ContactJointHandler::ContactJointHandler()
            {
                setName("ContactJointHandler");
            }
            
            void ContactJointHandler::setContactBounceMode(bool set)
            {
                mBounceMode = set;
            }

            void ContactJointHandler::setContactSlipMode(bool set)
            {
                mSlipMode = set;
            }

            void ContactJointHandler::setContactSoftERPMode(bool set)
            {
                mSoftERPMode = set;
            }

            void ContactJointHandler::setContactSoftCFMMode(bool set)
            {
                mSoftCFMMode = set;
            }

            void ContactJointHandler::setContactBounceValue(float bounce)
            {
                mBounceValue = bounce;
            }

            void ContactJointHandler::setMinBounceVel(float vel)
            {
                mMinBounceVel = vel;
            }
            
            void ContactJointHandler::setContactSlip(float slip)
            {
                mSlip = slip;
            }

            void ContactJointHandler::setContactSoftERP(float erp)
            {
                mSoftERP = erp;
            }

            void ContactJointHandler::setContactSoftCFM(float cfm)
            {
                mSoftCFM = cfm;
            }
    
        } /* namespace collider */
    } /* namespace device */
} /* namespace robot */
