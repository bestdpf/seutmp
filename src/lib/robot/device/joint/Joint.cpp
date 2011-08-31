/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "Joint.h"

namespace robot {
    namespace device {
        namespace joint {

            void Joint::attach(shared_ptr<RigidBody> body1, shared_ptr<RigidBody> body2)
            {
                mBody1 = body1;
                mBody2 = body2;
            }

            void Joint::setAnchor(const Vector3f& anchor)
            {
                mAnchor = anchor;
            }

            void Joint::setMaxMotorForce(int axis, float f)
            {
                mAxis[axis].maxMotorForce = f;
            }

            void Joint::setLowStopDeg(int axis, float deg)
            {
                mAxis[axis].lowStopDeg = deg;
            }

            void Joint::setHighStopDeg(int axis, float deg)
            {
                mAxis[axis].highStopDeg = deg;
            }

            void Joint::setCFM(int axis, float cfm)
            {
                mAxis[axis].CFM = cfm;
            }

            void Joint::setStopCFM(int axis, float cfm)
            {
                mAxis[axis].stopCFM = cfm;
            }

            void Joint::setStopERP(int axis, float erp)
            {
                mAxis[axis].stopERP = erp;
            }

            void Joint::setFudgeFactor(int axis, float ff)
            {
                mAxis[axis].fudgeFactor = ff;
            }

            void Joint::setBounce(int axis, float b)
            {
                mAxis[axis].bounce = b;
            }

            void Joint::setJointMaxSpeed1(float v)
            {
                mAxis[0].maxSpeed = v;
            }

            void Joint::setJointMaxSpeed2(float v)
            {
                mAxis[1].maxSpeed = v;
            }
			
			float Joint::getLowStopDeg(int axis) const
			{
                return mAxis[axis].lowStopDeg;
			}
			
			float Joint::getHighStopDeg(int axis) const
			{
                return mAxis[axis].highStopDeg;
			}

            float Joint::getRangeBisector(int axis) const
            {
                return calBisectorTwoAngles( getLowStopDeg(axis), getHighStopDeg(axis) );
            }
			
			float Joint::getJointMaxSpeed1 () const
			{
                return mAxis[0].maxSpeed;
            }
			
			float Joint::getJointMaxSpeed2 () const
			{
                return mAxis[1].maxSpeed;
            }

            void Joint::setDegreeOfFreedomId( int axis, unsigned int id )
            {
                mAxis[axis].id = id;
            }

            const Joint::DegreeOfFreedom& Joint::DOF( int axis ) const
            {
                return mAxis[axis];
            }

            void Joint::restricJointAngles(map<unsigned int, AngDeg>& angles) const
            {
                FOR_EACH( iter, mAxis ){
                    unsigned int id = iter->id;
                    angles[id] = iter->restricJointAngles(angles[id]);
                }
            }

            Vector3f Joint::mapAxis( int axis )
            {
                switch (axis){
                case 0: return Vector3f(1,0,0);
                    break;
                case 1: return Vector3f(0,1,0);
                    break;
                case 2: return Vector3f(0,0,1);
                    break;
                default:
                    cerr<<"[Joint Error] : do not know the mean of axis "<<axis<<endl;
                    return Vector3f(0,0,0);
                }
            }

            bool Joint::inRange( int axis, AngDeg ang ) const
            {
                const DegreeOfFreedom& dof = mAxis[axis];
                return dof.lowStopDeg-3 < ang && ang < dof.highStopDeg+3;
            }               
        } /* namespace joint */
    } /* namespace device */
} /* namespace robot */
