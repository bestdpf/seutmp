/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_DEVICE_JOINT_JOINT_H_
#define _ROBOT_DEVICE_JOINT_JOINT_H_

#include "../Device.h"
#include "../RigidBody.h"
#include <boost/shared_ptr.hpp>


namespace robot {
    namespace device {
        namespace joint {

            using namespace boost;
            
            class Joint : public Device
            {
            public:

                struct DegreeOfFreedom
                {
                    unsigned int id;
                    Vector3f axis;
                    float maxMotorForce;
                    float lowStopDeg;
                    float highStopDeg;
                    float CFM;
                    float stopCFM;
                    float stopERP;
                    float fudgeFactor;
                    float bounce;
                    float maxSpeed;

                    AngDeg restricJointAngles(AngDeg ang) const
                        {
                            return clamp(ang, lowStopDeg, highStopDeg);
                        }
                };
                
                void attach(shared_ptr<RigidBody> body1, shared_ptr<RigidBody> body2);

                void setAnchor(const Vector3f& anchor);
                
                const Vector3f& getAnchor() const { return mAnchor; }

                void setMaxMotorForce(int axis, float f);

                void setLowStopDeg(int axis, float deg);

                void setHighStopDeg(int axis, float deg);
					
				float getLowStopDeg(int axis) const;
					
				float getHighStopDeg(int axis) const;

                float getRangeBisector(int axis) const;

                void setCFM(int axis, float cfm);

                void setStopCFM(int axis, float cfm);

                void setStopERP(int axis, float erp);

                void setFudgeFactor(int axis, float ff);

                void setBounce(int axis, float b);

                void setJointMaxSpeed1(float v);

                void setJointMaxSpeed2(float v);
					
				float getJointMaxSpeed1() const;
					
				float getJointMaxSpeed2() const;

                void setDegreeOfFreedomId( int axis, unsigned int id );

                shared_ptr<const RigidBody> getBody1() const
                    {
                        return mBody1;
                    }

                shared_ptr<const RigidBody> getBody2() const
                    {
                        return mBody2;
                    }

                virtual void forwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const = 0;

                virtual void backwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles) const = 0;

                void restricJointAngles(map<unsigned int, AngDeg>& angles) const;

                const DegreeOfFreedom& DOF( int axis ) const;

                bool inRange(int axis, AngDeg ang) const;
                
            protected:

                static Vector3f mapAxis( int axis );
                
                shared_ptr<RigidBody> mBody1;
                shared_ptr<RigidBody> mBody2;
                Vector3f mAnchor;

                vector<DegreeOfFreedom> mAxis;
            };
            
        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */



#endif /* _ROBOT_DEVICE_JOINT_JOINT_H_ */
