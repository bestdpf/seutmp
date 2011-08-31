/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Humanoid.h"
#include "../device/Transform.h"
#include "../device/joint/HingeJoint.h"
#include "../device/joint/UniversalJoint.h"
#include "../device/effector/HingeJointEffector.h"
#include "../device/effector/UniversalJointEffector.h"
#include "../device/sensor/HingeJointSensor.h"
#include "../device/sensor/UniversalJointSensor.h"

namespace robot{
    namespace humanoid{

        using namespace boost;
        using namespace tree;
        using namespace device::joint;

        Humanoid::Humanoid(const string& name)
            :Robot(name)
        {
        }

        Humanoid::~Humanoid()
        {
        }

        void Humanoid::init()
        {
            makeSkeleton();
            cout<<*mBoneRoot<<endl;

            // sum the total mass
            mTotalMass = mBoneRoot->calculateTotalMass();
            cout<<"Total Mass: "<<mTotalMass<<endl;

            mDOF.clear();
            mJointSensorNames.clear();
            mHJEffectorNames.clear();
            mUJEffectorNames.clear();
            cacheBoneData(mBoneRoot);

            /*
            FOR_EACH( iter, mJointEffectorNames ){
                cout<<iter->first<<'\t'<<iter->second<<endl;
            }
            FOR_EACH( iter, mJointSensorNames ){
                cout<<iter->first<<'\t'<<iter->second<<endl;
            }
            */
        }

        void Humanoid::makeSkeleton()
        {
            mBoneRoot.reset();
            mBones.clear();

            int bn = 0;
            // get all the transform
            list<shared_ptr<Node<Device> > > tranList;
            listAll<Transform>(tranList);
            FOR_EACH(iter,tranList){
                shared_ptr<Bone> node = Bone::create(*iter);
                if ( 0 == node.get() ){
                    // it is not a bone
                    continue;
                }
                if ( 0 == mBoneRoot.get() ){ // the root node
                    mBoneRoot = node;
                }
                else{
                    if ( !mBoneRoot->attach(node) )
                        cerr<<"[Robot Error] can not attach "<<node->name()
                            <<" to properly body."<<endl;
                }
                node->setId(bn);
                mBones[bn] = node;
                bn++;
            }
        }

        void Humanoid::cacheBoneData(shared_ptr<Bone> b)
        {
            if ( NULL == b.get() ) return;
            int dof = mDOF.size();
            
            Joint* j = b->joint();
            if ( NULL != j ){
                HingeJoint* hj = dynamic_cast<HingeJoint*>(j);
                UniversalJoint* uj = dynamic_cast<UniversalJoint*>(j);
            
                if ( NULL != hj ){
                    // a hinge joint
                    hj->setDegreeOfFreedomId(0,dof);
                    mDOF[dof] = hj->DOF(0);
                    const HingeJointEffector* je = dynamic_cast<const HingeJointEffector*>( b->jointEffector() );
                    if ( NULL != je ){
                        // set the map from joint sensor to id
                        mHJEffectorNames[dof] = je->getName();
                    }
                    const HingeJointSensor* js = dynamic_cast<const HingeJointSensor*>( b->jointSensor() );
                    if ( NULL != js ){
                        // set the map from id to joint effector
                        mJointSensorNames[js->getName()] = dof;
                    }
                    dof++;
                }
                if ( NULL != uj ){
                    // a universal joint
                    uj->setDegreeOfFreedomId( 0, dof);
                    mDOF[dof] = uj->DOF(0);
                    uj->setDegreeOfFreedomId( 1, dof+1);
                    mDOF[dof] = uj->DOF(1);
                    const UniversalJointEffector* je = dynamic_cast<const UniversalJointEffector*>( b->jointEffector() );
                    if ( NULL != je ){
                        // set the map from joint sensor to id
                        mUJEffectorNames[dof] = je->getName();
                    }
                    const UniversalJointSensor* js = dynamic_cast<const UniversalJointSensor*>( b->jointSensor() );
                    if ( NULL != js ){
                        // set the map from id to joint effector
                        mJointSensorNames[js->getName()] = dof;
                        mJointSensorNames[js->getName()+"'"] = dof+1;
                    }
                    dof+=2;
                }
            }

            // call recursively
            cacheBoneData( b->child() );
            cacheBoneData( b->sister() );
        }

        const string& Humanoid::getJointSensorName(unsigned int dof) const
        {
            return findSensorNameById(mJointSensorNames, dof);
        }

        const string& Humanoid::getGyroRateName(unsigned int id) const
        {
            return findSensorNameById(mGyroRateNames, id);
        }
/////////////////////////////////////////
        const string& Humanoid::getAccelerometerName(unsigned int id) const
        {
            return findSensorNameById(mAccelerometerNames, id);
        }
//////////////////////////////////////
        const string& Humanoid::getTouchName(unsigned int id) const
        {
            return findSensorNameById(mTouchSensorNames, id);
        }

        const string& Humanoid::getForceResistanceName(unsigned int id) const
        {
            return findSensorNameById(mFRPNames, id);
        }
        
        int Humanoid::getJointSensorId(const string& name) const
        {
            return findSensorIdByName(mJointSensorNames, name);
        }

        int Humanoid::getGyroRateId(const string& name) const
        {
            return findSensorIdByName(mGyroRateNames, name);
        }
////////////////////////////////////////////////
        int Humanoid::getAccelerometerId(const string& name) const
        {
            return findSensorIdByName(mAccelerometerNames, name);
        }
 //////////////////////////////////////////////

        int Humanoid::getTouchId(const string& name) const
        {
            return findSensorIdByName(mTouchSensorNames, name);
        }

        int Humanoid::getForceResistanceId(const string& name) const
        {
            return findSensorIdByName(mFRPNames, name);
        }
        
        shared_ptr<const Bone> Humanoid::getBone( ESkeleton bId ) const
        {
            return getBone( getBoneId(bId) );
        }
        
        AngDeg Humanoid::calJointTurnAngle(unsigned int id,
                                           math::AngDeg ang0,
                                           math::AngDeg ang1) const
        {
            const Joint::DegreeOfFreedom& dof = mDOF.find(id)->second;
            ang0 = clamp( ang0, dof.lowStopDeg, dof.highStopDeg );
            ang1 = clamp( ang1, dof.lowStopDeg, dof.highStopDeg );
            return calClipAng( ang1, ang0 );
        }

        void Humanoid::forwardKinematics(ESkeleton bId, const TransMatrixf& trans,
                                         std::map<unsigned int, math::AngDeg>& angles,
                                         map<unsigned int, TransMatrixf>& mats) const
        {
            mats.clear();
            unsigned int id = getBoneId(bId);
            shared_ptr<const Bone> start = getBone(id);
            if ( NULL == start.get() ) return;
            mats[id] = trans;
            // 1. forward kinematics to the root
            TransMatrixf t = trans;
            start->backwardKinematics(t,angles,mats);

            // 2. forward kinematics to the leaf
            shared_ptr<const Bone> c = mBoneRoot->child();
            if ( NULL != c.get() ){
                c->forwardKinematics(t,angles,mats);
            }
        }

        shared_ptr<Bone> Humanoid::getBone( const string& name )
        {
            FOR_EACH( iter, mBones ){
                if ( name == iter->second->name() ){
                    return iter->second;
                }
            }
            return shared_ptr<Bone>();
        }

        shared_ptr<const Bone> Humanoid::getBone( unsigned int bId ) const
        {
            map<unsigned int, shared_ptr<Bone> >::const_iterator iter = mBones.find(bId);
            if ( mBones.end() == iter ) return shared_ptr<const Bone>();
            return iter->second;
        }

        unsigned int Humanoid::getBoneId( ESkeleton bId ) const
        {
            map<ESkeleton, unsigned int>::const_iterator iter = mSkeletonMap.find(bId);
            if ( mSkeletonMap.end() == iter ) return 0xffffffff; // strange value
            return iter->second;
        }

        bool Humanoid::inverseKinematics(const list<shared_ptr<const Bone> >& idx,
                                         const TransMatrixf& baseMat,
                                         const TransMatrixf& endMat,
                                         map<unsigned int, AngDeg>& angles,
                                         map<unsigned int, TransMatrixf>& mats) const
        {   
            const float lambda = 0.3f;
            mats.clear();
            
            float errlen;
            Vector6f err;
            Matrix6x6f J;
            unsigned int endId = idx.back()->getId();

            list<unsigned int> dofid;
            FOR_EACH( iter, idx ){
                dofid.push_back( (*iter)->joint()->DOF(0).id ); // only consider Hinge Joint here!!
            }
            
            for( int i=0; i<100; i++){
                forwardKinematics(idx, baseMat, angles, mats);
                const TransMatrixf& now = mats[ endId ];
                err = calcVWerr( endMat, now );
                errlen = pow2(err[0]) + pow2(err[1]) + pow2(err[2])
                    + ( pow2(err[3]) + pow2(err[4]) + pow2(err[5]) );
                if ( errlen < 1.0e-10 ){
                    return true;
                }
                calcJacobian(idx, mats, J);
                J.inv();
                // J.transpose();
                Vector6f dq = lambda * ( J * err );
                unsigned int j = 0;
                FOR_EACH(i, dofid){
                    float d = clamp(dq[j], -3.14f, 3.14f);
                    angles[*i] = normalizeAngle( angles[*i] + rad2Deg(d));
                    j++;
                }
            }
            return false;
        }

        Vector6f Humanoid::calcVWerr(const TransMatrixf& ref, const TransMatrixf& now) const
        {            
            Vector3f perr = ref.p() - now.p();
            Matrix3x3f Rerr = ref.R() * (~now.R());
            Vector3f werr = now.rotate( rot2omega(Rerr) );
            Vector6f err;
            err[0] = perr[0];
            err[1] = perr[1];
            err[2] = perr[2];
            err[3] = werr[0];
            err[4] = werr[1];
            err[5] = werr[2];
            return err;
        }

        void Humanoid::calcJacobian( const list<shared_ptr<const Bone> >& idx,
                                     const map<unsigned int, TransMatrixf>& mats,
                                     Matrix6x6f& J) const
        {
            shared_ptr<const Bone> endBone = idx.back();
            TransMatrixf target = mats.find( endBone->getId() )->second; // the key should be found!!
            endBone->calcJointGlobalMatFromBody(target);
            const Vector3f& targetP = target.p();
            unsigned int i = 0;
            FOR_EACH( iter, idx ){
                shared_ptr<const Bone> bone = *iter;
                TransMatrixf mat = mats.find( bone->getId() )->second; // the key should be found!!
                Vector3f a = bone->calcJointGlobalAxisFromBody(mat);
                bone->calcJointGlobalMatFromBody(mat);
                Vector3f e = targetP - mat.p();              
                Vector3f p = a.cross( e );
                J(0,i) = p[0];
                J(1,i) = p[1];
                J(2,i) = p[2];
                J(3,i) = a[0];
                J(4,i) = a[1];
                J(5,i) = a[2];
                i++;
            }
        }

        void Humanoid::forwardKinematics( const list<shared_ptr<const Bone> >& idx,
                                          const TransMatrixf& baseMat,
                                          map<unsigned int, AngDeg>& angles,
                                          map<unsigned int, TransMatrixf>& mats) const
        {
            TransMatrixf m = baseMat;
            FOR_EACH( iter, idx ){
                (*iter)->forwardKinematics(m, angles);
                mats[ (*iter)->getId() ] = m;
            }
        }
        
        bool Humanoid::findRoute(ESkeleton from, ESkeleton to,
                                 list<shared_ptr<const Bone> >& idx) const
        {
            map<ESkeleton, unsigned int>::const_iterator iterF = mSkeletonMap.find(from);
            if ( mSkeletonMap.end() == iterF ) return false;

            map<ESkeleton, unsigned int>::const_iterator iterT = mSkeletonMap.find(to);
            if ( mSkeletonMap.end() == iterT ) return false;

            return findRoute(iterF->second, iterT->second, idx);
        }

        bool Humanoid::findRoute(unsigned int from, unsigned int to,
                                 list<shared_ptr<const Bone> >& idx) const
        {
            idx.clear();
            map<unsigned int, shared_ptr<Bone> >::const_iterator iter = mBones.find(to);
            if ( mBones.end() == iter ) return false;
            shared_ptr<const Bone> b = iter->second;

            while ( from != to ){
                idx.push_front(b);
                b = b->parent();
                if ( NULL == b.get() ) return false;
                to = b->getId();
            }
            
            return true;
        }

        bool Humanoid::legInverseKinematics(bool isLeft,
                                            const TransMatrixf& torsoMat,
                                            const TransMatrixf& footMat,
                                            map<unsigned int, AngDeg>& angles) const
        {
            list<shared_ptr<const Bone> > idx;
            if ( !findRoute(TORSO,  isLeft?L_FOOT:R_FOOT, idx) ) return false;

            map<unsigned int, TransMatrixf> mats;
            return inverseKinematics(idx, torsoMat, footMat, angles, mats);
        }

        Vector3f Humanoid::calcCenterOfMass(const map<unsigned int, TransMatrixf>& mats) const
        {
            Vector3f com(0,0,0);
            float mass = 0;
            FOR_EACH( iter, mats ){
                float m = getBone(iter->first)->body()->mass();
                mass += m;
                com += (iter->second.pos()*m);
            }
            com *= (1.0f/mass);
            return com;
        }

        Humanoid::ESkeleton Humanoid::getBoneESkeleton( unsigned int id ) const
        {
            map<ESkeleton, unsigned int>::const_iterator iter = findMapBySecondValue(mSkeletonMap,id);
            if ( mSkeletonMap.end() == iter ) return ILLEGAL;
            return iter->first;
        }

        const Vector3f& Humanoid::getFootSupportBias(bool isLeft) const
        {
            return isLeft?mLeftFootSupportBias:mRightFootSupportBias;
        }

        void Humanoid::restricJointAngles(std::map<unsigned int, math::AngDeg>& angles) const
        {
            FOR_EACH(iter, mDOF){
                iter->second.restricJointAngles(angles[iter->first]);
            }
        }
        
    } /* namespace humanoid */
} /* namespace robot */
