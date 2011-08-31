/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_HUMANOID_BONE_H_
#define _ROBOT_HUMANOID_BONE_H_

#include "../device/joint/Joint.h"
#include "../device/RigidBody.h"
#include "../device/collider/Collider.h"
#include "../device/effector/Effector.h"
#include "../device/sensor/Sensor.h"
#include "Node.hpp"


namespace robot{
    namespace humanoid{

        using namespace boost;
        using namespace tree;
        using namespace robot::device;
        using namespace robot::device::joint;
        using namespace robot::device::collider;
        using namespace robot::device::effector;
        using namespace robot::device::sensor;
        
        class Bone : public BaseNode<Bone,BINARY_TREE>
        {
            
        public:
            
            Bone():mJoint(NULL),mJointEffector(NULL),mJointSensor(NULL),
                   mBody(NULL),mCollider(NULL),mId(-1){};
            
            static shared_ptr<Bone> create(shared_ptr<Node<Device> > dev);
            
            shared_ptr<const RigidBody> getMotherBody() const;

            const RigidBody* body() const
                {
                    return mBody;
                }

            const Joint* joint() const { return mJoint; }
            Joint* joint() { return mJoint; }

            const Effector* jointEffector() const
                {
                    return mJointEffector;
                }

            const Sensor* jointSensor() const
                {
                    return mJointSensor;
                }

            const string& name() const
                {
                    return mDevice->getName();
                }

            const string& getName() const
                {
                    return mDevice->getName();
                }

            void setId(unsigned int id ) { mId = id; }

            unsigned int getId() const { return mId; }
            
            /** 
             * find the matched body, attach the bone to the correct
             * place
             * 
             * @param bone the attached bone
             * 
             * @return if the operator successful, i.e. find the body
             */
            bool attach(shared_ptr<Bone> bone);

            const Collider* collider() const
                {
                    return mCollider;
                }

            void forwardKinematics(TransMatrixf& m,
                                   const map<unsigned int, AngDeg>& angles) const;

            void forwardKinematics(const TransMatrixf& m, const map<unsigned int, AngDeg>& angles,
                                   map<unsigned int, TransMatrixf>& mats) const;

            void backwardKinematics(TransMatrixf& m,
                                    const map<unsigned int, AngDeg>& angles) const;
            
            void backwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles,
                                    map<unsigned int, TransMatrixf>& mats) const;



            const TransMatrixf& getGlobalMat() const;

            /** 
             * calculate the joint global matrix according to body matrix
             * 
             * @param bodyMat the given body matrix
             * 
             * @return the global matrix of joint
             */
            void calcJointGlobalMatFromBody(TransMatrixf& mat) const;
            void calcJointGlobalMatFromParent(TransMatrixf& m) const;

            Vector3f calcJointGlobalAxisFromBody(const TransMatrixf& bodyMat) const;
            
            TransMatrixf getJointGlobalMat() const;

            void setMatrixM2J(const TransMatrixf& m);

            void restricJointAngles(map<unsigned int, AngDeg>& angles) const;

            float calculateTotalMass() const;
            
        private:
            /** the root node of this bone */
            shared_ptr<const Node<Device> > mDevice;

            /** cache pointer to the joint */
            Joint* mJoint;
            const Effector* mJointEffector;
            const Sensor* mJointSensor;

            /** cache pointer to the body */
            const RigidBody* mBody;
            
            //const DragController* mDragCtr;
            const Collider* mCollider;

            /// the transform matrix from mother body to joint
            TransMatrixf mTmj;
            /// the transfrom matrix from joint to body
            TransMatrixf mTjb;

            unsigned int mId;
        };

        std::ostream& operator<<(std::ostream &stream, const Bone& p);
        
    } /* namespace humanoid */
} /* namespace robot */

#endif /* _ROBOT_HUMANOID_BONE_H_ */
