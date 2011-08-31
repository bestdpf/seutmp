/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Kick.h"
#include "MoveCoM.h"
#include "core/WorldModel.h"
#include "TurnLeg.h"
#include "AccelerateFoot.h"
#include "BackLeg.h"

namespace task{

    using namespace boost;
    using namespace math;
    using namespace action;
	using namespace serversetting;
    
	
	DEFINE_STATIC_GRAPHIC_LOGGER(Kick)
	
    Kick::Kick( const math::Vector2f& target,const KickMode& mode,
				bool isLeftLeg,Task* primary)
        :Task(-1, primary) // -1: as soon as possiable
    {
		BEGIN_ADD_STATIC_LOG_LAYER(Kick) 
        ADD_LOG_LAYER("kick"); 
        ADD_LOG_LAYER("goto"); 
		ADD_LOG_LAYER("camera")
        
        END_ADD_STATIC_LOG_LAYER(Kick)
		
		mKickMode = mode;
		mIsLeftLeg = isLeftLeg;
		
		Vector2f ballPos;
		ballPos = WM.getBallGlobalPos();
		mTarget = calAmendTarget(target,ballPos,mode);
		
		
		// create the sequence tasks for start kicking
		//1.squat
		float high = 3.4578 - 1.00/2 - 0.55/2 + 0.25 - 0.55/2 - 0.3 +1.1;
		high += 0.05;
        Vector3f squat(0, 0, high);
        shared_ptr<Task> msquat ( new MoveCoM(squat, 0.5, this) );
        mSubTaskList.push_back(msquat);
		if(mKickMode == KICK_MODE_HIGH)
		{
			shared_ptr<Task> msquat2 ( new MoveCoM(squat, 0.5, this) );
        	mSubTaskList.push_back(msquat2);
		}
		//shared_ptr<Task> msquat2 ( new MoveCoM(squat, 0.5, this) );
		//mSubTaskList.push_back(msquat2);
		
        //2. move CoM to left or right side
		Vector3f com;
		const static float torsoMoveX = 0.35;
		if(mIsLeftLeg)
        	com = Vector3f(torsoMoveX * 0.5, 0, high);
		else
			com = Vector3f(-torsoMoveX * 0.5, 0, high);
		
        shared_ptr<Task> mcom ( new MoveCoM(com, 0.5, this ) );
        mSubTaskList.push_back(mcom);
		
		//3.back leg
		shared_ptr<Task> mBackLeg ( new BackLeg(this));
		mSubTaskList.push_back(mBackLeg);
       
		//4.turn Leg
		shared_ptr<Task> mTurnLeg ( new TurnLeg(this));
		mSubTaskList.push_back(mTurnLeg);
		
		//5.accelerate foot
		shared_ptr<Task> mAccelerateFoot ( new AccelerateFoot(this));
		mSubTaskList.push_back(mAccelerateFoot);
		
    }

	
	void Kick::generateDesiredJointAngles(const TransMatrixf& hip,const TransMatrixf& footL,const TransMatrixf& footR,
											perception::JointPerception& desiredPose)
    {
        Vector6f qR;// = robot::Kinematics::analysisInverseLeg(hip, footR, 0.39, 1.35, 0.7765, SS.mFootHeight*3 ); 	// *3
        Vector6f qL;// = robot::Kinematics::analysisInverseLeg(hip, footL, -0.39, 1.35, 0.7765, SS.mFootHeight*3 );	// *3
#if 0
        // left leg
        desiredPose[JID_LEG_R_1].setAngle( qR[0] );
        desiredPose[JID_LEG_R_2].setAngle( qR[1] );
        desiredPose[JID_LEG_R_3].setAngle( qR[2] );
        desiredPose[JID_LEG_R_4].setAngle( qR[3] );
        desiredPose[JID_LEG_R_5].setAngle( qR[4] );
        desiredPose[JID_LEG_R_6].setAngle( qR[5] );
        // right leg
        desiredPose[JID_LEG_L_1].setAngle( qL[0] );
        desiredPose[JID_LEG_L_2].setAngle( qL[1] );
        desiredPose[JID_LEG_L_3].setAngle( qL[2] );
        desiredPose[JID_LEG_L_4].setAngle( qL[3] );
        desiredPose[JID_LEG_L_5].setAngle( qL[4] );
        desiredPose[JID_LEG_L_6].setAngle( qL[5] );
#endif	
        // left arm
        //mDesiredPose[JID_ARM_L_1].setAngle( qR[1] );
        // right arm
        //mDesiredPose[JID_ARM_R_1].setAngle( qL[1] );

        // log
        LOG_PRINTF("Kick","LEG_L_1: %f LEG_R_1: %f",qL[0],qR[0]);
        LOG_PRINTF("Kick","LEG_L_2: %f LEG_R_2: %f",qL[1],qR[1]);
        LOG_PRINTF("Kick","LEG_L_3: %f LEG_R_3: %f",qL[2],qR[2]);
        LOG_PRINTF("Kick","LEG_L_4: %f LEG_R_4: %f",qL[3],qR[3]);
        LOG_PRINTF("Kick","LEG_L_5: %f LEG_R_5: %f",qL[4],qR[4]);
        LOG_PRINTF("Kick","LEG_L_6: %f LEG_R_6: %f",qL[5],qR[5]);        
    }
	
	
	//修正踢球点
	
	Vector2f Kick::calAmendTarget(Vector2f target,Vector2f ballPos,KickMode mode)
	{
		Vector2f amendTarget;
		AngDeg amendAng =0;
		switch(mode)
		{
			case KICK_MODE_NORAML:
				break;
			case KICK_MODE_FAST:
				if(mIsLeftLeg)
					amendAng = -4.1680;
				else
					amendAng = 4.1680;	
				break;
			case KICK_MODE_LOW:
				if(mIsLeftLeg)
					amendAng = -12.73;
				else
					amendAng = 12.73;					
				break;
						
			case KICK_MODE_LOW2:
				if(mIsLeftLeg)
					amendAng = -10.243;
				else
					amendAng = 10.243;
				break;
			case KICK_MODE_LOW3:
				if(mIsLeftLeg)
					amendAng = -4.33;
				else
					amendAng = 4.33;
				break;				
			case KICK_MODE_HIGH:
				if(mIsLeftLeg)
					amendAng = -16.3;
				else
					amendAng = 16.3;
				break;	
			case KICK_MODE_PASS_1:
				if(mIsLeftLeg)
					amendAng = 5.17;
				else
					amendAng = -5.17;
				break;			
			case KICK_MODE_PASS_2:
				if(mIsLeftLeg)
					amendAng = -9.55;
				else
					amendAng = 9.55;
				break;					
			case KICK_MODE_PASS_3:
				if(mIsLeftLeg)
					amendAng = -7.6;
				else
					amendAng = 7.6;
				break;							
			default:
				break;
			
		}		
		float length = (target - ballPos).length();
		Vector2f ball2target = target - ballPos;
		AngDeg relBall2Target = atan2Deg(ball2target[1],ball2target[0]);
		AngDeg relBall2Amend = relBall2Target - amendAng;
		relBall2Amend = normalizeAngle(relBall2Amend);
		amendTarget = ballPos + Vector2f(cosDeg(relBall2Amend),sinDeg(relBall2Amend))*length;
		LOG_PRINTF("kick","target = %f,%f, amendTarget = %f,%f",target[0],target[1],amendTarget[0],amendTarget[1]);
		return amendTarget;
	}
	
	
	//解方程：((x,y,z) - ballPos).length() = ball2footLength
	//       (ballPos - (x,y)).direction = relBall2Target;
	//ballPos.z = 0.4
	void Kick::calKickFootLastPos(Vector2f target,Vector2f ballPos,Vector3f& foot)
	{
		float ball2footLength = 1.61; 
		switch(mKickMode)
		{
			case KICK_MODE_FAST:
				ball2footLength = 1.61;
				break;
			case KICK_MODE_LOW:
				ball2footLength = 1.25;
				break;
			case KICK_MODE_LOW2:
				ball2footLength = 1.4;
				break;
			case KICK_MODE_LOW3:
				ball2footLength = 1.61;
				break;
			case KICK_MODE_HIGH:
				ball2footLength = 1.61;
				break;
			case KICK_MODE_PASS_1:
				ball2footLength = 1.61;
				break;
			case KICK_MODE_PASS_2:
				ball2footLength = 1.4;
				break;
			case KICK_MODE_PASS_3:
				ball2footLength = 1.5;
				break;
			case KICK_MODE_TEST:
				ball2footLength = 1.7;
				break;
			
			default:
				break;				
		}
		
		const static float minFootHeight = 0;//TODO SS.mFootHeight*0.5;
		
		//foot[2] = minFootHeight + 0.5; //set z pos;
		Vector2f ball2target = target - ballPos;
		AngDeg relBall2Target = atan2Deg(ball2target[1],ball2target[0]);
		
		float x,y,z;
		z = minFootHeight + 0.5;
		
		if(fabs(relBall2Target) < 90)
			x = ballPos[0] - sqrt((pow2(ball2footLength) - pow2(0.4 - z))/(1 + pow2(tanDeg(relBall2Target))));
		else
			x = ballPos[0] + sqrt((pow2(ball2footLength) - pow2(0.4 - z))/(1 + pow2(tanDeg(relBall2Target))));
		y = ballPos[1] - tanDeg(relBall2Target) * (ballPos[0] - x);
		LOG_PRINTF("kick","x = %f,y = %f,z = %f",x,y,z);
		float length;
		length = (Vector3f(x,y,z) - Vector3f(ballPos[0],ballPos[1],0.4)).length();
		Vector2f xy2Ball = ballPos - Vector2f(x,y);
		LOG_PRINTF("kick","relBall2Target = %f , xy2Ball = %f,length = %f",relBall2Target,xy2Ball.angle(),length);
		foot = Vector3f(x,y,z);
		
		TransMatrixf mat = WM.getMyOriginTrans();
		mat.inverseTransfer(foot);
		foot = mat.p(); 
		LOG_PRINTF("kick","footR :%f,%f,%f",foot[0],foot[1],foot[2]);
	}
    
	
	bool Kick::isGoToRightKickPlace(const Vector2f& target,const Vector2f& ballPos,KickMode mode,bool isLeftLeg)
	{	
		const Vector2f& myPos = WM.getMyOrigin2D();
		
		KickParameter p;		
		p = calKickParameter(target,ballPos,mode,isLeftLeg);
		//Vector2f bestp2ball = ballPos - p.movePos;
		//AngDeg relBestp2ball = atan2Deg(bestp2ball[1],bestp2ball[0]);
		//Vector2f my2ball = ballPos - myPos;
		//AngDeg relMy2ball = atan2Deg(my2ball[1],my2ball[0]);
		
		//LOG_BLUE_LINE("kick",Vector3f(myPos[0],myPos[1],1),Vector3f(myPos[0],myPos[1],1) + Vector3f(cosDeg(relMy2ball),sinDeg(relMy2ball),0)*20);
		//LOG_LINE("kick",Vector3f(p.movePos[0],p.movePos[1],1),Vector3f(p.movePos[0],p.movePos[1],1) + Vector3f(cosDeg(relBestp2ball),sinDeg(relBestp2ball),0)*20,logger::Color::YELLOW);
		AngDeg bodyAng = WM.getMyFaceDirection();
		AngDeg disAng = bodyAng - p.bodyAng;
		disAng = normalizeAngle(disAng);
		
		if(fabs(disAng) > p.errAng)
		{	
			LOG_PRINTF("kick","|disAng| > p.errAng , disAng = %f , p.errAng = %f",disAng,p.errAng);
			LOG_FLUSH;
			return false;
		}
		
		
		//float length = (p.movePos - ballPos).length();
		//float lengthNow = (myPos - ballPos).length();
		float length = (p.movePos - myPos).length();
		if(length > p.errLength)
		{	
			LOG_RED_SPHERE("kick",myPos,0.1);
			LOG_BLUE_SPHERE("kick",p.movePos,0.1);
			LOG_PRINTF("kick","no near , length = %f,p.errLength=%f",length,p.errLength);
			LOG_FLUSH;
			return false;
		}
		
		
		
		LOG_PRINTF("kick","disAng = %f , p.errAng = %f",disAng,p.errAng);
		LOG_PRINTF("kick","length = %f,p.errLength=%f",length,p.errLength);
		
		return true;
		
	}
	
	bool Kick::isGoToRightKickPlace2(const Vector2f& target,
                                     const Vector2f& ballPos,
                                     KickMode mode,
                                     bool isLeftLeg)
	{	
		const Vector2f& myPos = WM.getMyOrigin2D();
		KickParameter p = calKickParameter(target,ballPos,mode,isLeftLeg);
		
		LOG_PRINTF("goto","SuppotFoot = %d",WM.getMySupportFoot());	
		AngDeg bodyAng = WM.getMyFaceDirection();
		AngDeg disAng = bodyAng - p.bodyAng;
		disAng = normalizeAngle(disAng);
		if(fabs(disAng) > p.errAng)
		{	
			LOG_PRINTF("goto","bodyAng out of range , bodyAng = %f,p.bodyAng = %f,disAng = %f , p.errAng = %f",bodyAng,p.bodyAng,disAng,p.errAng);
			LOG_FLUSH;
			return false;
		}
		
		
		float length = (p.movePos - ballPos).length();
		float lengthNow = (myPos - ballPos).length();
		//float length = (p.movePos - myPos).length();
		if(fabs(lengthNow - length) > p.errLength)
		{	
			LOG_PRINTF("goto","length out of range , lengthNow = %f,length = %f,p.errLength=%f",lengthNow,length,p.errLength);
			LOG_FLUSH;
			return false;
		}
		
		Vector2f bestp2ball = ballPos - p.movePos;
		AngDeg relBestp2ball = atan2Deg(bestp2ball[1],bestp2ball[0]);
		Vector2f my2ball = ballPos - myPos;
		AngDeg relMy2ball = atan2Deg(my2ball[1],my2ball[0]);
		AngDeg disAng2 = relMy2ball - relBestp2ball;
		disAng2 = normalizeAngle(disAng2);
		if(fabs(disAng2) > p.errAng)
		{
			LOG_PRINTF("goto","ang2 out of range ,disAng2 = %f",disAng2);
			LOG_FLUSH;
			return false;
		}
		
		//LOG_BLUE_LINE("kick",Vector3f(myPos[0],myPos[1],1),Vector3f(myPos[0],myPos[1],1) + Vector3f(cosDeg(relMy2ball),sinDeg(relMy2ball),0)*20);
		//LOG_LINE("kick",Vector3f(p.movePos[0],p.movePos[1],1),Vector3f(p.movePos[0],p.movePos[1],1) + Vector3f(cosDeg(relBestp2ball),sinDeg(relBestp2ball),0)*20,logger::Color::YELLOW);
		LOG_PRINTF("goto","disAng = %f ,disAng2 = %f, p.errAng = %f",disAng,disAng2,p.errAng);
		LOG_PRINTF("goto","length = %f,lengthNow = %f,p.errLength=%f",length,lengthNow,p.errLength);
		/*ofstream out("kick_goto.txt");
          out<<disAng<<" "<<disAng2<<" "<<length - lengthNow<<" "<<endl;*/
		if(isLeftLeg && WM.getMySupportFoot() > 0 )
			return true;
		if(!isLeftLeg && WM.getMySupportFoot() < 0)
			return true;
		//WM.isTouch(FRID_LEFT_FOOT)
		LOG_FLUSH;
		return false;
		
	}
	
	float Kick::calKickHeight(Vector2f target,Vector2f ballPos,KickMode mode)
	{
		Vector2f ball2target = target - ballPos;
		float x,y =0;
		x = ball2target.length();
		switch(mode)
		{
			case KICK_MODE_NORAML:
				
				break;
			case KICK_MODE_FAST:
				y = -0.005299*x*x + 0.349*x + 0.01035;
				break;			
			case KICK_MODE_HIGH:
				y = -0.06596*x*x + 1.493*x + 0.8518;
				break;
			default:
				break;
			
		}
		
		
		
		
		return y;		
	}
	
		//isLeftLeg = true,use left leg to kick;
	KickParameter Kick::calKickParameter(const Vector2f& target,
                                         const Vector2f& ballPos,
                                         KickMode mode,
                                         bool isLeftLeg)
	{
		KickParameter p;
		p.errAng = 5;
		p.errLength = 0.2;
		float backLength = 1.1;
		float rightLength = 0.39;
		Vector2f kickPoint;
		Vector2f ball2target = target - ballPos;
		Vector2f movePos;
		AngDeg direction;
		ball2target.normalize();
		direction = atan2Deg(ball2target[1],ball2target[0]);
		//Vector2f myPos= WM.getMyGlobalCenter();
		
		AngDeg ang2;
		ang2 = normalizeAngle(direction - 90);
		
		switch(mode)
		{
			case KICK_MODE_NORAML:
				
				break;
			case KICK_MODE_FAST:
				backLength = 1.1;//1.05;
				rightLength = 0.4;//0.35;
				p.errLength = 0.05;
				p.errAng = 22.0;
				break;		
			
			case KICK_MODE_HIGH:
				backLength = 1.18;//1.15;//1.05;
				rightLength = 0.4;//0.35;
				p.errLength = 0.05;
				p.errAng = 22.0;
				break;
			case KICK_MODE_LOW:			
			case KICK_MODE_LOW2:
			case KICK_MODE_LOW3:
			case KICK_MODE_PASS_1:
			case KICK_MODE_PASS_2:
			case KICK_MODE_PASS_3:
			case KICK_MODE_TEST:
				backLength = 1.1;//1.05;
				rightLength = 0.4;//0.35;
				p.errLength = 0.05;
				p.errAng = 22.0;
				break;				
			default:
				backLength = 1.05;//1.05;
				rightLength = 0.4;//0.35;
				p.errLength = 0.2;
				p.errAng = 40.0;
				break;
			
		}		
		
		if(!isLeftLeg)
			rightLength = -rightLength;
			
				
		kickPoint = ballPos - ball2target*backLength;
		
		LOG_GREEN_SPHERE("camera",kickPoint,0.1);
		movePos[0] = kickPoint[0] + rightLength * cosDeg(ang2);
		movePos[1] = kickPoint[1] + rightLength * sinDeg(ang2);
		p.movePos = movePos;
		//Vector2f movePos2ball = ballPos - movePos;
		//direction = atan2Deg(movePos2ball[1],movePos2ball[0]);
		p.bodyAng = direction;
		return p;
		//LOG_RED_SPHERE("testKick",Vector3f(kickPoint[0],kickPoint[1],0.4),0.4);
		//LOG_BLUE_SPHERE("testKick",Vector3f(movePos[0],movePos[1],0.4),0.4);
	}
	

	KickParameter Kick::calKickParameter() const
    {
        return calKickParameter(mTarget, WM.getBallGlobalPos2D(),
                                mKickMode, mIsLeftLeg);
    }

    bool Kick::isAchieveable() const
    {
        return isGoToRightKickPlace(mTarget, WM.getBallGlobalPos2D(),
                                     mKickMode, mIsLeftLeg);
    }
	
} // namespace task
