/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Bone.h"
#include "../device/Transform.h"


namespace robot{
    namespace humanoid{

        using namespace robot::device;
        
        /**
         * make a bone by the name, the format in the device is
         * Transform(name) - Body - DragController
         *                 + Collider - ContactJointHandler
         *                 + Joint
         */
        shared_ptr<Bone> Bone::create(shared_ptr<Node<Device> > dev)
        {
            shared_ptr<Bone> bone = BaseNode<Bone,BINARY_TREE>::create();
            bone->mDevice = dev;
            // cache the body pointer
            shared_ptr<const Node<Device> > bNode = dev->getTheOneChild<RigidBody>();
            if ( NULL == bNode.get() ){
                // no body, it is not a Bone
                return shared_ptr<Bone>();
            }
            bone->mBody = dynamic_cast<const RigidBody*>( bNode->data().get() );
            
            // cache the joint pointer
            shared_ptr<Node<Device> > jNode = dev->getTheOneChild<Joint>();
            if ( NULL != jNode.get() ){
                bone->mJoint = dynamic_cast<Joint*>( jNode->data().get() );
                // cache the joint effector
                shared_ptr<const Node<Device> > jeNode = jNode->getTheOneChild<Effector>();
                if ( NULL != jeNode.get() ){
                    bone->mJointEffector = dynamic_cast<const Effector*>( jeNode->data().get() );
                }
                // cache the joint sensor
                shared_ptr<const Node<Device> > jsNode = jNode->getTheOneChild<Sensor>();
                if ( NULL != jsNode.get() ){
                    bone->mJointSensor = dynamic_cast<const Sensor*>( jsNode->data().get() );
                }
            }
            
            // cache the collider pointer
            shared_ptr<const Node<Device> > cNode = dev->getTheOneChild<Collider>();
            if ( NULL != cNode.get() ){
                bone->mCollider = dynamic_cast<const Collider*>( cNode->data().get() );
            }
            if ( NULL != bone->mJoint ){
                // update the trans-matrix from joint to body
                bone->mTjb.identity();
                bone->mTjb.pos() = -(bone->mJoint->getAnchor());
            }
            
            bone->mTmj.identity();
            
            return bone;
        }

        shared_ptr<const RigidBody> Bone::getMotherBody() const
        {
            if ( NULL == mJoint ){
                // no mother body
                return shared_ptr<const RigidBody>();
            }
            shared_ptr<const RigidBody> b1 = mJoint->getBody1();
            if ( b1.get() != mBody )
                return b1;
            return mJoint->getBody2();
        }

        bool Bone::attach(shared_ptr<Bone> bone)
        {
            const RigidBody* motherBody = bone->getMotherBody().get();
            if ( mBody == motherBody ){ // it is!
                addChild(bone);
                // update the child's trans-matrix from mother's body to joint
                TransMatrixf tmj = getGlobalMat(); // global matrix of mother
                TransMatrixf localMat = bone->getJointGlobalMat(); // global matrix of joint
                tmj.inverseTransfer(localMat); // local matrix of joint in mother
                bone->setMatrixM2J(tmj);
                return true;
            }
            
            shared_ptr<Bone> sis = sister();
            if ( 0 != sis.get() ){ // try to attach to sister
                if ( sis->attach(bone) ){
                    return true;
                }
            }

            shared_ptr<Bone> ch = child();
            if ( 0 != ch.get() ){ // try to attach to child
                if ( ch->attach(bone) ){
                    return true;
                }
            }
            
            // can not find matched body
            return false;
        }

        void Bone::setMatrixM2J(const TransMatrixf& m )
        {
            mTmj = m;
        }

        void Bone::calcJointGlobalMatFromBody(TransMatrixf& mat) const
        {
            mat.transfer(!mTjb); // global matrix of joint
        }

        void Bone::calcJointGlobalMatFromParent(TransMatrixf& m) const
        {
            m.transfer(mTmj);
        }

        Vector3f Bone::calcJointGlobalAxisFromBody(const TransMatrixf& bodyMat) const
        {
            if ( NULL == mJoint ) return Vector3f(0,0,0);
            return bodyMat.rotate(mJoint->DOF(0).axis);
        }
        
        TransMatrixf Bone::getJointGlobalMat() const
        {
            TransMatrixf m = getGlobalMat(); // global matrix of body
            calcJointGlobalMatFromBody(m);
            return m;
        }

        std::ostream& operator<<(std::ostream &stream, const Bone& p)
        {
            stream<<p.name();
            shared_ptr<const Bone> child = p.child();
            if ( 0 != child.get() )
                stream<<" - "<<*child;
            shared_ptr<const Bone> sister = p.sister();
            if ( 0 != sister.get() )
                stream<<"\n\t"<<*sister;
            return stream;
        }

        const TransMatrixf& Bone::getGlobalMat() const
        {
            shared_ptr<const Transform> t =
                shared_dynamic_cast<const Transform>( mDevice->data() );
            if ( 0 == t.get() ){
                cerr<<"[Robot Error] can not get "<<name()
                    <<"'s initial global matrix!"<<endl;
            }
            return t->getGlobalMat();
        }

        void Bone::forwardKinematics(TransMatrixf& m,
                                     const map<unsigned int, AngDeg>& angles) const
        {
            m.transfer(mTmj);
            if ( NULL != mJoint ){
                mJoint->forwardKinematics(m, angles);
            }
            m.transfer(mTjb);
        }

        void Bone::forwardKinematics(const TransMatrixf& m,
                                     const map<unsigned int, AngDeg>& angles,
                                     map<unsigned int, TransMatrixf>& mats) const
        {
            TransMatrixf t = m;
            forwardKinematics(t,angles);
            mats[mId]=t;

            shared_ptr<const Bone> s = sister();
            if ( NULL != s.get() ){
                s->forwardKinematics(m,angles,mats);
            }

            shared_ptr<const Bone> c = child();
            if ( NULL != c.get() ){
                c->forwardKinematics(t,angles,mats);
            }
        }

        void Bone::backwardKinematics(TransMatrixf& m,
                                      const map<unsigned int, AngDeg>& angles) const
        {
            shared_ptr<const Bone> p = parent();
            if ( NULL == p.get() ) return;
            
            m.transfer(!mTjb);
            if ( NULL != mJoint ){
                mJoint->backwardKinematics(m, angles);
            }
            m.transfer(!mTmj);
        }
        
        void Bone::backwardKinematics(TransMatrixf& m, const map<unsigned int, AngDeg>& angles,
                                      map<unsigned int, TransMatrixf>& mats) const
        {
            shared_ptr<const Bone> p = parent();
            if ( NULL == p.get() ) return;
            
            m.transfer(!mTjb);
            if ( NULL != mJoint ){
                mJoint->backwardKinematics(m, angles);
            }
            m.transfer(!mTmj);
            
            mats[p->getId()]=m;
            p->backwardKinematics(m,angles,mats);
        }

        void Bone::restricJointAngles(map<unsigned int, AngDeg>& angles) const
        {
            if ( NULL == mJoint ) return;
            mJoint->restricJointAngles(angles);
        }

        float Bone::calculateTotalMass() const
        {
            float m = mBody->mass();

            list<shared_ptr<const Bone> > chs = children();
            FOR_EACH(iter, chs){
                m += (*iter)->calculateTotalMass();
            }
            
            return m;
        }      
    } /* namespace humanoid */
} /* namespace robot */
