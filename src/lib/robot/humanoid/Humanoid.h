/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef _ROBOT_HUMANOID_H_
#define _ROBOT_HUMANOID_H_

#include "../Robot.h"
#include "Bone.h"

namespace robot{
    namespace humanoid{

        using namespace std;
        
        class Humanoid : public Robot
        {
        public:
            enum ESkeleton{
                TORSO = 0,
                HEAD,
                L_HAND,
                R_HAND,
                L_HIP,
                R_HIP,
                L_THIGH,
                R_THIGH,
                L_SHANK,
                R_SHANK,
                L_FOOT,
                R_FOOT,
                ILLEGAL
            };
            
            Humanoid(const string& name);
            virtual ~Humanoid();
            
            static Humanoid& getSingleton()
                { return *dynamic_cast<Humanoid*>(mTheRobot);}

            shared_ptr<const Bone> getBone() const { return mBoneRoot; }

            shared_ptr<const Bone> getBone( unsigned int id ) const;

            shared_ptr<Bone> getBone( const string& name );

            unsigned int getBoneId( ESkeleton bId ) const;

            ESkeleton getBoneESkeleton( unsigned int id ) const;

            shared_ptr<const Bone> getBone( ESkeleton bId ) const;

            typedef map<string, unsigned int> TSensorNameIdMap;
            /** 
             * helper function for get sensor' id by sensor name
             * 
             * @param m the map from sensor name to id
             * @param name the sensor name
             * 
             * @return the id of sensor, -1 means no such sensor
             */
            static int findSensorIdByName(const TSensorNameIdMap& m,
                                          const string& name)
                {
                    TSensorNameIdMap::const_iterator iter = m.find(name);
                    if ( m.end() == iter ) return -1;
                    return iter->second;
                }

            static const string& findSensorNameById(const TSensorNameIdMap& m,
                                                    unsigned int id)
                {
                    FOR_EACH( iter, m ){
                        if ( iter->second == id ){
                            return iter->first;
                        }
                    }
                    static string errStr = "UnknowSensor";
                    return errStr;
                }
            
            const string& getJointSensorName(unsigned int dof) const;

            const string& getGyroRateName(unsigned int id) const;
 //////////////////////////////////////////////////////////
            const string& getAccelerometerName(unsigned int id) const;
/////////////////////////////////////////////
            const string& getTouchName(unsigned int id) const;

            const string& getForceResistanceName(unsigned int id) const;

            int getJointSensorId(const string& name) const;

            int getGyroRateId(const string& name) const;
 //////////////////////////////////////////////////////////
            int getAccelerometerId(const string& name) const;
//////////////////////////////////////////////////////////
            int getTouchId(const string& name) const;

            int getForceResistanceId(const string& name) const;

            size_t degreeOfFreedom() const 
                { return mDOF.size(); }

            size_t boneNums() const
                {
                    return mBones.size();
                }

            /** 
             * calculate the angle if the joint want to turn from ang0 to ang1
             * 
             * @param id the id of joint
             * @param ang0 the begin angle
             * @param ang1 the end angle
             * 
             * @return the turning angle
             */
            AngDeg calJointTurnAngle(unsigned int id,
                                     math::AngDeg ang0,
                                     math::AngDeg ang1) const;

            void restricJointAngles(std::map<unsigned int, math::AngDeg>& angles) const;

            void forwardKinematics(ESkeleton bId, const TransMatrixf& trans,
                                   std::map<unsigned int, math::AngDeg>& angles,
                                   map<unsigned int, TransMatrixf>& mats) const;

            const map<unsigned int, string>& getHingeJointEffectors() const
                { return mHJEffectorNames; }

            const map<unsigned int, string>& getUniversalJointEffectors() const
                { return mUJEffectorNames; }

            void forwardKinematics( const list<shared_ptr<const Bone> >& idx,
                                    const TransMatrixf& baseMat,
                                    map<unsigned int, AngDeg>& angles,
                                    map<unsigned int, TransMatrixf>& mats) const;
            
            bool inverseKinematics(const list<shared_ptr<const Bone> >& idx,
                                   const TransMatrixf& baseMat,
                                   const TransMatrixf& endMat,
                                   map<unsigned int, AngDeg>& angles,
                                   map<unsigned int, TransMatrixf>& mats) const;

            bool inverseKinematics2(const list<shared_ptr<const Bone> >& idx,
                                   const TransMatrixf& baseMat,
                                   const TransMatrixf& endMat,
                                   map<unsigned int, AngDeg>& angles,
                                   map<unsigned int, TransMatrixf>& mats) const;

            void calcJacobian( const list<shared_ptr<const Bone> >& idx,
                               const map<unsigned int, TransMatrixf>& mats,
                               Matrix6x6f& jacobian) const;

            Vector6f calcVWerr(const TransMatrixf& ref, const TransMatrixf& now) const;

            virtual bool legInverseKinematics(bool isLeft,
                                              const TransMatrixf& torsoMat,
                                              const TransMatrixf& footMat,
                                              std::map<unsigned int, math::AngDeg>& angles) const;

            /** 
             * find the bones route (list) from *from* to *to*, this
             * function can only handle downward, such as from torso
             * to foot
             * 
             * @param from the start bone id
             * @param to the end bone id
             * @param idx the list of bones
             * 
             * @return if there is a such route
             */
            bool findRoute(ESkeleton from, ESkeleton to,
                           list<shared_ptr<const Bone> >& idx) const;
            
            /** 
             * find the bones route (list) from *from* to *to*, this
             * function can only handle downward, such as from torso
             * to foot
             * 
             * @param from the start bone id
             * @param to the end bone id
             * @param idx the list of bones
             * 
             * @return if there is a such route
             */
            bool findRoute(unsigned int from, unsigned int to,
                           list<shared_ptr<const Bone> >& idx) const;

            /** 
             * calculate the center of mass of the robot
             * 
             * @param mats the transform matrix of bones
             * 
             * @return the CoM
             */
            Vector3f calcCenterOfMass(const map<unsigned int, TransMatrixf>& mats) const;

            /** 
             * get the bias from foot to the robot's origin
             * 
             * @param isLeft left foot or right foot
             * 
             * @return the vector from foot to the robot's origin
             */
            const Vector3f& getFootSupportBias(bool isLeft) const;

            float getMinFootHeight() const { return mMinFootHeight; }

            float getHalfFeetWidth() const { return mHalfFeetWidth; }

            float getShoulderWidth() const { return mShoulderWidth; }

            float getTotalMass() const { return mTotalMass; }

            float getLegMass() const { return mLegMass; }

            float getArmMass() const { return mArmMass; }

            const Vector3f& getFootSize() const { return mFootSize; }

            const Vector3f& getBoundBoxSize() const { return mBoundBoxSize; }
            
        protected:
            virtual void init();

            /** create the skeleton from devices */
            void makeSkeleton();

            void cacheBoneData(shared_ptr<Bone> b);
            
            /** the root of skeleton of the humanoid robot */
            shared_ptr<Bone> mBoneRoot;

            /** caches the bone */
            map<unsigned int, shared_ptr<Bone> > mBones;

            /** enumeration <--> bone id */
            map<ESkeleton, unsigned int> mSkeletonMap;
            
            /** map from Id to joint DOF */
            map<unsigned int, Joint::DegreeOfFreedom> mDOF;
            
            /** map from joint sensor's name to Id */
            TSensorNameIdMap mJointSensorNames;
            
            /** map from Gyro rate to Id */
            TSensorNameIdMap mGyroRateNames;
            ////////////////////////////add by allen
             /** map from Accelerometer to Id */
            TSensorNameIdMap mAccelerometerNames;
             /////////////////////////////////////////////
            /** map from touch sensor to Id */
            TSensorNameIdMap mTouchSensorNames;

            /** map from FRP to Id */
            TSensorNameIdMap mFRPNames;

            /** map from Id to joint sensor's name */
            map<unsigned int,string> mHJEffectorNames;
            map<unsigned int,string> mUJEffectorNames;

            Vector3f mLeftFootSupportBias;
            Vector3f mRightFootSupportBias;
            float mMinFootHeight;
            /** the half width between two feet */
            float mHalfFeetWidth;

            /** the width of the shoulder */
            float mShoulderWidth;

            Vector3f mBoundBoxSize;

            /// mass
            float mTotalMass;   /**< total mass */
            float mLegMass;     /**< mass of leg */
            float mArmMass;     /**< mass of arm */

            Vector3f mFootSize;
        };
        
    } /* namespace humanoid */
} /* namespace robot */

#define HUMANOID robot::humanoid::Humanoid::getSingleton()

#endif /* _ROBOT_HUMANOID_H_ */
