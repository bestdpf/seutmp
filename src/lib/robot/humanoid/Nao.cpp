/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Nao.h"

namespace robot { namespace humanoid {

        Nao::Nao()
            :Humanoid("rsg/agent/nao/nao.rsg")
        {
            init();
        }

        Nao::~Nao()
        {
        }

        void Nao::init()
        {
            Humanoid::init();

            mSkeletonMap.clear();
            mSkeletonMap[TORSO] = getBone("body")->getId();
            mSkeletonMap[HEAD] = getBone("head")->getId();
            mSkeletonMap[L_HAND] = getBone("llowerarm")->getId();
            mSkeletonMap[R_HAND] = getBone("rlowerarm")->getId();
            mSkeletonMap[L_HIP] = getBone("lhip1")->getId();
            mSkeletonMap[R_HIP] = getBone("rhip1")->getId();
            mSkeletonMap[L_THIGH] = getBone("lthigh")->getId();
            mSkeletonMap[R_THIGH] = getBone("rthigh")->getId();
            mSkeletonMap[L_SHANK] = getBone("lshank")->getId();
            mSkeletonMap[R_SHANK] = getBone("rshank")->getId();
            mSkeletonMap[L_FOOT] = getBone("lfoot")->getId();
            mSkeletonMap[R_FOOT] = getBone("rfoot")->getId();

            // sensor names
            // this can be set automatically, todo?
            mGyroRateNames.clear();
            mGyroRateNames["torso"] = getBone("body")->getId();
            //////////////////////////////add by allen 2010.3.15
            mAccelerometerNames.clear();
            mAccelerometerNames["torso"] = getBone("body")->getId();

            //////////////////////////////////////////////////
            mFRPNames.clear();
            mFRPNames["lf"] = getBone("lfoot")->getId();
            mFRPNames["rf"] = getBone("rfoot")->getId();

            // length of leg, used in IK
            TransMatrixf kneeMat = getBone(L_SHANK)->getGlobalMat();
            getBone(L_SHANK)->calcJointGlobalMatFromBody(kneeMat);
            TransMatrixf hipMat = getBone(L_THIGH)->getGlobalMat();
            getBone(L_THIGH)->calcJointGlobalMatFromBody(hipMat);
            TransMatrixf ankleMat = getBone(L_FOOT)->getGlobalMat();
            getBone(L_FOOT)->calcJointGlobalMatFromBody(ankleMat);
            mThighLength = ( hipMat.p() - kneeMat.p() ).length();
            mShankLength = ( kneeMat.p() - ankleMat.p() ).length();
            mMaxLegLength = ( hipMat.p() - ankleMat.p() ).length();

            mMinFootHeight = getBone(R_FOOT)->body()->size().z() * 0.5f;
            // foot support bias
            const Vector3f& lfp = getBone(L_FOOT)->getGlobalMat().p();
            const Vector3f& rfp = getBone(R_FOOT)->getGlobalMat().p();
            mHalfFeetWidth = ( rfp.x() - lfp.x() ) * 0.5;
            mLeftFootSupportBias = Vector3f(mHalfFeetWidth,0,-mMinFootHeight);
            mRightFootSupportBias = Vector3f(-mHalfFeetWidth,0,-mMinFootHeight);
            // shoulder
            mShoulderWidth = ( getBone("rshoulder")->getGlobalMat().p().x()
                               - getBone("lshoulder")->getGlobalMat().p().x() ) * 0.5f;
            mLegMass = getBone("lhip1")->calculateTotalMass();
            mArmMass = getBone("lshoulder")->calculateTotalMass();

            mFootSize = getBone(L_FOOT)->body()->size();
            
            mBoundBoxSize.x() = mShoulderWidth*2 + getBone("rshoulder")->body()->size().x();
            mBoundBoxSize.y() = getBone(TORSO)->body()->size().y();
            mBoundBoxSize.z() = getBone(HEAD)->getGlobalMat().p().z()
                - getBone(L_FOOT)->getGlobalMat().p().z()
                + getBone(HEAD)->body()->size().z() * 0.5f
                + mMinFootHeight;
        }

        bool Nao::legInverseKinematics(bool isLeft,
                                  const TransMatrixf& torsoMat,
                                  const TransMatrixf& footMat,
                                  std::map<unsigned int, math::AngDeg>& angles) const
        {
            shared_ptr<const Bone> hip = getBone(isLeft?L_HIP:R_HIP);
            if ( NULL == hip.get() ) return false;
            shared_ptr<const Bone> foot = getBone(isLeft?L_FOOT:R_FOOT);
            if ( NULL == foot.get() ) return false;
            
            TransMatrixf mat = torsoMat;
            TransMatrixf ankleMat = footMat;
            hip->calcJointGlobalMatFromParent(mat);
            foot->calcJointGlobalMatFromBody(ankleMat);
            
            Vector3f vah = ankleMat.p() - mat.p();
            float len = vah.length();
            if ( len > mMaxLegLength ){
                vah *= ( mMaxLegLength/len );
                Vector3f vfa = footMat.p() - ankleMat.p();
                TransMatrixf newFoot = footMat;
                newFoot.p() = mat.p() + vah + vfa;
                return legIK( isLeft, torsoMat, newFoot, angles, 2 );
                // return legIK2( isLeft, torsoMat, newFoot, angles);
            }
            // cout<<"torso:\n"<<torsoMat<<'\n'<<"foot:\n"<<footMat<<endl;
            return legIK( isLeft, torsoMat, footMat, angles, 2 ); // limit the max computation time
            // return legIK2( isLeft, torsoMat, footMat, angles);
        }
        
        bool Nao::legIK(bool isLeft,
                        const TransMatrixf& torsoMat,
                        const TransMatrixf& footMat,
                        std::map<unsigned int, math::AngDeg>& angles,
                        int count) const
        {   
            count--;
            if ( count < 0 ){
                return false;
            }
            
            list<shared_ptr<const Bone> > idx;
            if ( !findRoute(TORSO,  isLeft?L_FOOT:R_FOOT, idx) ) return false;

            list<shared_ptr<const Bone> >::iterator iter = idx.begin();
            shared_ptr<const Bone> hip1 = *iter;
            ++iter;
            shared_ptr<const Bone> hip2 = *iter;
            ++iter;
            shared_ptr<const Bone> thigh = *iter;
            ++iter;
            shared_ptr<const Bone> shank = *iter;
            ++iter;
            shared_ptr<const Bone> ankle = *iter;
            ++iter;
            shared_ptr<const Bone> foot = *iter;

            const Joint* hip1Joint = hip1->joint();
            const Joint* shankJoint = shank->joint();
            const Joint* hip2Joint = hip2->joint();
            const Joint* thighJoint = thigh->joint();
            const Joint* ankleJoint = ankle->joint();
            const Joint* footJoint = foot->joint();
            unsigned int hip1Id = hip1Joint->DOF(0).id;
            unsigned int kneeId = shankJoint->DOF(0).id;
            unsigned int hip2Id = hip2Joint->DOF(0).id;
            unsigned int thighId = thighJoint->DOF(0).id;
            unsigned int footId = footJoint->DOF(0).id;
            unsigned int ankleId = ankleJoint->DOF(0).id;
            
            map<unsigned int, TransMatrixf> mats;
            if ( !inverseKinematics(idx, torsoMat, footMat, angles, mats) ){
                // cout<<count<<" failed ... recompute!"<<endl;
                angles[hip1Id] = hip1Joint->getRangeBisector(0);
                return legIK(isLeft, torsoMat, footMat, angles, count);
            }

            if ( !hip1Joint->inRange( 0, angles[hip1Id] ) ){
                AngDeg hip1Ang = normalizeAngle( angles[hip1Id] + 180 );
                if ( !hip1Joint->inRange( 0, hip1Ang ) )
                {
                    hip1Ang = -angles[hip1Id];
                    // cout<<"hip1 "<<angles[hip1Id]<<" rotate 180, but still not in range!"<<endl;
                }
                angles[hip1Id] = hip1Ang;
                TransMatrixf m = torsoMat;
                m.transfer( hip1Joint->DOF(0).axis, hip1Ang);
                TransMatrixf thighMat = mats[ thigh->getId() ];
                Matrix3x3f R = thighMat.R() * (~m.R());
                angles[hip2Id] = atan2Deg( -R(0,2), R(0,0) );
                angles[thighId] = atan2Deg( -R(2,1), R(1,1) );
            }

            if ( !hip2Joint->inRange( 0, angles[hip2Id]) ){
                angles[hip2Id] = normalizeAngle( angles[hip2Id] + 180 );
            }
            
            TransMatrixf ankleMat = footMat;
            foot->calcJointGlobalMatFromBody(ankleMat);
            const Vector3f& anklePos = ankleMat.p();
            const Vector3f& hipPos = mats[hip1->getId()].p();
            Vector3f r = footMat.inverseRotate( hipPos - anklePos );
            float C = r.length();
            float c3 = ( pow2(mThighLength) + pow2(mShankLength) - pow2(C) )
                    /(2.0*mThighLength*mShankLength);
            AngDeg kneeAng = acosDeg(c3);
            if ( !shankJoint->inRange( 0, angles[kneeId] ) ){
                angles[kneeId] = kneeAng - 180 + 2.386;
                //@note: the position of joints are not the same in y direction !!!
            }
            
            AngDeg alpha = asinDeg( (mThighLength/C) * sinDeg(kneeAng) );
            angles[ankleId] = atan2Deg(r[1],(sign(r[2])*sqrt( pow2(r[0]) + pow2(r[2]))) ) + alpha;
            if ( !footJoint->inRange( 0, angles[footId] )  ){
                AngDeg footAng = -atan2Deg(r[0],r[2]);
                if ( footAng > 90 ) angles[footId] = footAng - 180;
                else if ( footAng < -90 ) angles[footId] = footAng + 180;
                else angles[footId] = footAng;
            }
            if ( !footJoint->inRange( 0, angles[footId] )  ){
                angles[footId] = normalizeAngle( angles[footId] + 180 );
            }

            // thigh
            TransMatrixf m = torsoMat;
            m.transfer( hip1Joint->DOF(0).axis, angles[hip1Id] );
            m.transfer( hip2Joint->DOF(0).axis, angles[hip2Id] );

            TransMatrixf Troll, Tpitch;
            Troll.rotationX( -angles[kneeId] - angles[ankleId] );
            Tpitch.rotationY( -angles[footId] );
            Matrix3x3f R = Troll.R() * Tpitch.R() * footMat.R() * (~m.R());
            angles[thighId] = atan2Deg( R(1,2), R(1,1) );
            
            if ( !hip1Joint->inRange( 0, angles[hip1Id] ) ){
                // cout<<"hip1 still error ["<<hip1Id<<"]"<<angles[hip1Id]<<endl;
                angles[hip1Id] = hip1Joint->getRangeBisector(0);
                return legIK(isLeft, torsoMat, footMat, angles, count);
            }
            if ( !hip2Joint->inRange( 0, angles[hip2Id] ) ){
                // cout<<"hip2 still error ["<<hip2Id<<"]="<<angles[hip2Id]<<endl;
                angles[hip2Id] = hip2Joint->getRangeBisector(0);
                return legIK(isLeft, torsoMat, footMat, angles, count);
            }
            if ( !thighJoint->inRange( 0, angles[thighId] ) ){
                // cout<<"thigh still error"<<endl;
                angles[thighId] = thighJoint->getRangeBisector(0);
                return legIK(isLeft, torsoMat, footMat, angles, count);
            }
            if ( !shankJoint->inRange( 0, angles[kneeId] ) ){
                // cout<<"knee still error ["<<kneeId<<"]="<<angles[kneeId]<<endl;
                angles[kneeId] = shankJoint->getRangeBisector(0);
                return legIK(isLeft, torsoMat, footMat, angles, count);
            }
            if ( !ankleJoint->inRange( 0, angles[ankleId] ) ){
                // cout<<"ankle still error ["<<ankleId<<"]="<<angles[ankleId]<<endl;
                angles[ankleId] = ankleJoint->getRangeBisector(0);
                return legIK(isLeft, torsoMat, footMat, angles, count);
            }
            if ( !footJoint->inRange( 0, angles[footId] ) ){
                // cout<<"foot still error"<<endl;
                angles[footId] = footJoint->getRangeBisector(0);
                return legIK(isLeft, torsoMat, footMat, angles, count);
            }
            return true;
        }


        bool Nao::legIK2(bool isLeft,
                         const TransMatrixf& torsoMat,
                         const TransMatrixf& footMat,
                         std::map<unsigned int, math::AngDeg>& angles) const
        {
            list<shared_ptr<const Bone> > idx;
            if ( !findRoute(TORSO,  isLeft?L_FOOT:R_FOOT, idx) ) return false;

            list<shared_ptr<const Bone> >::iterator iter = idx.begin();
            shared_ptr<const Bone> hip1 = *iter;
            ++iter;
            shared_ptr<const Bone> hip2 = *iter;
            ++iter;
            shared_ptr<const Bone> thigh = *iter;
            ++iter;
            shared_ptr<const Bone> shank = *iter;
            ++iter;
            shared_ptr<const Bone> ankle = *iter;
            ++iter;
            shared_ptr<const Bone> foot = *iter;

            
            const Joint* shankJoint = shank->joint();
            const Joint* ankleJoint = ankle->joint();
            const Joint* footJoint = foot->joint();
            
            unsigned int kneeId = shankJoint->DOF(0).id;
            unsigned int footId = footJoint->DOF(0).id;
            unsigned int ankleId = ankleJoint->DOF(0).id;

            TransMatrixf ankleMat = footMat;
            foot->calcJointGlobalMatFromBody(ankleMat);
            TransMatrixf hipMat = torsoMat;
            hip1->calcJointGlobalMatFromParent(hipMat);
            const Vector3f& anklePos = ankleMat.p();
            const Vector3f& hipPos = hipMat.p();
            Vector3f r = footMat.inverseRotate( hipPos - anklePos );
            float C = r.length();
            float c3 = ( pow2(mThighLength) + pow2(mShankLength) - pow2(C) )
                    /(2.0*mThighLength*mShankLength);
            AngDeg kneeAng = acosDeg(c3);

            angles[kneeId] = kneeAng - 180 + 2.386;
            //@note: the position of joints are not the same in y direction !!!
            
            AngDeg alpha = asinDeg( (mThighLength/C) * sinDeg(kneeAng) );
            angles[ankleId] = atan2Deg(r[1],(sign(r[2])*sqrt( pow2(r[0]) + pow2(r[2]))) ) + alpha;
            
            AngDeg footAng = -atan2Deg(r[0],r[2]);
            if ( footAng > 90 ) angles[footId] = footAng - 180;
            else if ( footAng < -90 ) angles[footId] = footAng + 180;
            else angles[footId] = footAng;
            
            TransMatrixf thighMat = footMat;
            foot->backwardKinematics(thighMat, angles);
            ankle->backwardKinematics(thighMat, angles);
            shank->backwardKinematics(thighMat, angles);
            
            return inverseHip(hip1,hip2,thigh,
                              torsoMat, thighMat, angles);
        }

        bool Nao::inverseHip(boost::shared_ptr<const Bone> hip1,
                             boost::shared_ptr<const Bone> hip2,
                             boost::shared_ptr<const Bone> thigh,
                             const TransMatrixf& torsoMat,
                             const TransMatrixf& thighMat,
                             std::map<unsigned int, math::AngDeg>& angles) const
        {
            const Joint* hip1Joint = hip1->joint();
            const Joint* hip2Joint = hip2->joint();
            const Joint* thighJoint = thigh->joint();
            
            unsigned int hip1Id = hip1Joint->DOF(0).id;
            unsigned int hip2Id = hip2Joint->DOF(0).id;
            unsigned int thighId = thighJoint->DOF(0).id;
            
            TransMatrixf m[3];
            boost::shared_ptr<const Bone> bone[3];
            bone[0] = hip1;
            bone[1] = hip2;
            bone[2] = thigh;
            
            const Vector3f& targetP = thighMat.p();

            Vector6f err;
            Matrix3x6f J;
            // TMatrix<float,6,3> Jt;
            for(int i=0; i<1000; i++){
                
                m[0]= torsoMat;
                hip1->forwardKinematics(m[0], angles);
                m[1]=m[0];
                hip2->forwardKinematics(m[1], angles);
                m[2]=m[1];
                thigh->forwardKinematics(m[2], angles);

                err = calcVWerr( thighMat, m[2] );
                
                if ( err.squareLength() < 1.0e-4 ){
                    return true;
                }

                for(int k=0;k<3;k++){
                    Vector3f a = bone[k]->calcJointGlobalAxisFromBody(m[k]);
                    Vector3f e = targetP - m[k].p();
                    Vector3f p = a.cross(e);
                    J(k,0) = p[0]; //Jt(0,k) = p[0];
                    J(k,1) = p[1]; //Jt(1,k) = p[1];
                    J(k,2) = p[2]; //Jt(2,k) = p[2];
                    J(k,3) = a[0]; //Jt(3,k) = a[0];
                    J(k,4) = a[1]; //Jt(4,k) = a[1];
                    J(k,5) = a[2]; //Jt(5,k) = a[2];
                }

                // Matrix3x3f M = J*Jt;
                // M.inv();
                // const float lambda = 1.0f;
                // Vector3f dq = lambda * ( M * J * err );
                Vector3f dq = J * err;
                angles[hip1Id] += dq[0];
                angles[hip2Id] += dq[1];
                angles[thighId] += dq[2];
                hip1Joint->restricJointAngles( angles );
                hip2Joint->restricJointAngles( angles );
                thighJoint->restricJointAngles( angles );                
            }
            return false;
        }
        
        
    } } /* namespace robot:: huamnoid */
