/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
 
#include "configuration/Configuration.h"
#include "core/WorldModel.h"
#include "FootAdjuster.h"

namespace controller {
    using namespace std;
    using namespace boost;
    using namespace serversetting;
    
FootAdjuster::FootAdjuster()
{

BEGIN_ADD_LOG_LAYER(FootAdjuster)
    ADD_LOG_LAYER("output");
    //ADD_LOG_LAYER("adjustKnee");
 
END_ADD_LOG_LAYER(FootAdjuster)
}
	
void FootAdjuster::adjustOneFoot(const perception::JointPerception& p,
                                 const map<unsigned int, TransMatrixf>& m,
                                 shared_ptr<action::JointAction> act,
                                 unsigned int shankID, unsigned int jidX,
                                 unsigned int jidY, bool adjustY)
{
	// note here R^(-1) = R(T)
	// ...
	const Matrix3x3f& R = m.find(shankID)->second.R();
	AngDeg angY = atan2Deg( R(0,2), R(2,2) );
    // restric the desired joint angle
    //angY = SS.restricJointAngle(jidY,angY);
	AngDeg angX;
    float cy = cosDeg(angY);
    if ( fabs(cy) > 0.707 )
        angX = atan2Deg( -R(1,2), R(2,2)/cy );
    else
        angX = atan2Deg( -R(1,2), R(0,2)/sinDeg(angY) );
    // restric the desired joint angle
	//angX = SS.restricJointAngle(jidX,angX);
    AngDeg angX0 = p.jointAng(jidX);
	AngDeg angY0 = p.jointAng(jidY);
	
	const float k = 0.5/serversetting::sim_step*0.5f;
	
	angX = calClipAng(angX,angX0)*k;
	angY = calClipAng(angY,angY0)*k;

    static const float maxV = 300;
    angX = clamp(angX,-maxV,maxV);
    angY = clamp(angY,-maxV,maxV);
    
	act->set(jidX,angX);
    if ( adjustY )
        act->set(jidY,angY);
	
	//LOG_PRINTF("output"," %s = %f; %s = %f",SS.getPerceptorName(jidX).c_str(),angX,SS.getPerceptorName(jidY).c_str(),angY);
}

// void FootAdjuster::adjust(int foot, shared_ptr<action::JointAction> act, bool adjustY)
// {
// 	const perception::Perception& p = WM.lastPerception();
// 	const map<unsigned int, TransMatrixf>& m = WM.getBoneTrans();
	
// 	// using touch perceptor
// 	/*if ( foot < 0 && p.touch().val(TID_FOOT_LEFT) == 0 )
// 	{
// 		adjustOneFoot(p.joints(),m,act,JID_LEG_L_4,JID_LEG_L_5,JID_LEG_L_6);
// 	}
// 	if ( foot > 0 && p.touch().val(TID_FOOT_RIGHT) == 0 )
// 	{
// 		adjustOneFoot(p.joints(),m,act,JID_LEG_R_4,JID_LEG_R_5,JID_LEG_R_6);
// 	}*/
	
// 	// using force resistance perceptor
// 	if ( foot < 0 && !p.forceResistance().isTouch(FRID_LEFT_FOOT) )
// 	{
//         //adjustKnee(p.joints(),m,act,JID_LEG_L_1,JID_LEG_L_2,JID_LEG_L_3,JID_LEG_L_4);
// 		// adjustOneFoot(p.joints(),m,act,JID_LEG_L_4,JID_LEG_L_5,JID_LEG_L_6, adjustY);
// 	}
// 	if ( foot > 0 && !p.forceResistance().isTouch(FRID_RIGHT_FOOT) )
// 	{
//         //adjustKnee(p.joints(),m,act,JID_LEG_R_1,JID_LEG_R_2,JID_LEG_R_3,JID_LEG_R_4);
// 		// adjustOneFoot(p.joints(),m,act,JID_LEG_R_4,JID_LEG_R_5,JID_LEG_R_6, adjustY);
// 	}
//     LOG_FLUSH;
// }

/** @todo the function write stiffness, need to modify for flexibility */
// void FootAdjuster::adjustKnee(const perception::JointPerception& p, const TJointTransMap& m, shared_ptr<action::JointAction> act,
//                 JointID hipID, JointID thighID1, JointID thighID2, JointID kneeID)
// {
    // LOG_PRINT("adjustKnee","start adjustKnee");

    // // a simple solution
    // const TJointTransMap::const_iterator thighIter = m.find(thighID1);
    // if ( m.end() == thighIter ) return;
    // TransMatrixf knee = thighIter->second;
    // SS.jointLinker(kneeID).calcAxisByParent(knee);
    // const float ankleHeightD = SS.mFootHeight*3+0.05+SS.mFootHeight*1.0;
    // const float shankLength = SS.mShankHeight+0.05-SS.mFootHeight*2.5;
    // float relHeight = knee.pos().z() - ankleHeightD;
    // float angKneeD = 0;
    // if ( relHeight < -shankLength )
    // {
    //     angKneeD = -180;
    // }
    // else if ( relHeight < shankLength )
    // {
    //     angKneeD = asinDeg(relHeight/shankLength) - 90;
    // }
    // angKneeD -= p.jointAng(thighID1);
    // angKneeD = normalizeAngle(angKneeD);

    // AngDeg ang0 = p.jointAng(kneeID);
	// const float k = 0.5/serversetting::sim_step;
	// // restric the desired joint angle
	// AngDeg kneeAng = SS.restricJointAngle(kneeID,angKneeD);
	// kneeAng = calClipAng(kneeAng,ang0);
	// act->set(kneeID,kneeAng*k);

    // LOG_RED_BOX("adjustKnee",knee,Vector3f(0.2,0.2,0.2));
    // knee = thighIter->second;
    // SS.jointLinker(kneeID).forwardKinematics(knee,angKneeD);
    // LOG_RED_BOX("adjustKnee",knee,Vector3f(0.2,0.2,0.2));
    // LOG_PRINTF("adjustKnee","shankLength=%f, relHeight=%f, ang0=%f, angKneeD=%f",shankLength,relHeight,ang0,angKneeD);
// }


} //namespace controller
