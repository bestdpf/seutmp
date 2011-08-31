/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "controller/Timing.h"
#include "AccelerateFoot.h"

namespace task{

    using namespace std;
    using namespace boost;
    using namespace math;
    using namespace serversetting;
	using namespace controller;


	DEFINE_STATIC_GRAPHIC_LOGGER(AccelerateFoot)

    AccelerateFoot::AccelerateFoot(Task* primary)
        :Task(1, primary)
    {

        BEGIN_ADD_STATIC_LOG_LAYER(AccelerateFoot)
        ADD_LOG_LAYER("kick");
        ADD_LOG_LAYER("goto");

        END_ADD_STATIC_LOG_LAYER(AccelerateFoot)

    }

    shared_ptr<action::Action> AccelerateFoot::perform()
    {
		Task::perform();

		Kick* p = dynamic_cast<Kick*>(mParentTask);
		if ( 0==p )
		{
			shared_ptr<Action> act;
			cout<<"AccelerateFoot change error"<<endl;
			return act;
		}


		LOG_PRINTF("kick","do kick pose accelerate foot");
		const static float minFootHeight = 0;//TODO SS.mFootHeight*0.5;
        //const static float torsoMoveX = 0.35;
        mPoseParameter.hip.identity();
        mPoseParameter.hip.p().z() = 3.4578 - 1.00/2 - 0.55/2 + 0.25 - 0.55/2 - 0.3;
        mPoseParameter.hip.p().z() += 0.05;
        mPoseParameter.footL.identity(); mPoseParameter.footR.identity();
        mPoseParameter.footL.p().x() = -0.3900;		mPoseParameter.footR.p().x() = 0.3900;
        mPoseParameter.footL.p().z() = minFootHeight;	mPoseParameter.footR.p().z() = minFootHeight;

		p->generateDesiredJointAngles(mPoseParameter.hip,mPoseParameter.footL,mPoseParameter.footR,mDesiredJoints);
		mActionCache = controller::Timing::control(WM.lastPerception(), mDesiredJoints, getRemainTime());

		Vector2f posBall;
        posBall = WM.getBallGlobalPos();
		shared_ptr<JointAction> jact = shared_static_cast<JointAction>(mActionCache);
		AngDeg ang2,ang4;
		float time2,time4;
		ang2 = 60;
		ang4 = 0;
		time2 = 0.04;
		time4 = 0.04;
		static int cyc = 0;
		if(isFirstDoAction())
			cyc =0;


		switch(p->getKickMode())
		{
			case Kick::KICK_MODE_NORAML:

				break;
			case Kick::KICK_MODE_FAST:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_FAST");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.04;
				time4 = 0.04;
				break;
			case Kick::KICK_MODE_LOW:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_LOW");
				ang2 = 60;
				ang4 = 20;
				time2 = 0.08;
				time4 = 0.05;
				break;
			case Kick::KICK_MODE_LOW2:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_LOW2");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.06;
				time4 = 0.045;
				break;
			case Kick::KICK_MODE_LOW3:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_LOW3");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.04;
				time4 = 0.04;
				if(cyc <=2)
				{
					if(p->getIsLeftLeg())
						jact->set(/*JID_LEG_L_5 TODO*/0,-400);
					else
						jact->set(/*JID_LEG_R_5 TODO*/0,-400);
					LOG_PRINTF("kick"," set JID_LEG_R_5 ");
				}
				break;
			case Kick::KICK_MODE_HIGH:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_HIGH");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.04;
				time4 = 0.03;
				if(cyc <=2)
				{
					if(p->getIsLeftLeg())
						jact->set(/*JID_LEG_L_5 TODO*/0,400);
					else
						jact->set(/*JID_LEG_R_5 TODO*/0,400);
					LOG_PRINTF("kick"," set JID_LEG_R_5 ");
				}
				break;
			case Kick::KICK_MODE_PASS_1:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_PASS_1");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.04;
				time4 = 0.04;
				if(cyc <=2)
				{
					if(p->getIsLeftLeg())
						jact->set(/*JID_LEG_L_5 TODO*/0,600);
					else
						jact->set(/*JID_LEG_R_5 TODO*/0,600);
					LOG_PRINTF("kick"," set JID_LEG_R_5 ");
				}
				break;
			case Kick::KICK_MODE_PASS_2:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_PASS_2");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.06;
				time4 = 0.06;
				if(cyc <=2)
				{
					if(p->getIsLeftLeg())
						jact->set(/*JID_LEG_L_5 TODO*/0,-1000);
					else
						jact->set(/*JID_LEG_R_5 TODO*/0,-1000);
					LOG_PRINTF("kick"," set JID_LEG_R_5 ");
				}
				break;
			case Kick::KICK_MODE_PASS_3:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_PASS_3");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.05;
				time4 = 0.05;
				if(cyc <=2)
				{
					if(p->getIsLeftLeg())
						jact->set(/*JID_LEG_L_5 TODO*/0,-1200);
					else
						jact->set(/*JID_LEG_R_5 TODO*/0,-1200);
					LOG_PRINTF("kick"," set JID_LEG_R_5 ");
				}
				break;
			case Kick::KICK_MODE_TEST:
				LOG_PRINTF("kick"," kick mode  = KICK_MODE_TEST");
				ang2 = 60;
				ang4 = 0;
				time2 = 0.05;
				time4 = 0.05;
				if(cyc <=2)
				{
					if(p->getIsLeftLeg())
						jact->set(/*JID_LEG_L_5 TODO*/0,-1200);
					else
						jact->set(/*JID_LEG_R_5 TODO*/0,-1200);
					LOG_PRINTF("kick"," set JID_LEG_R_5 ");
				}
				break;
			default:
				break;

		}

		cyc++;

		if(!(p->getIsLeftLeg()))
		{
			Vector2f posBall;
			TransMatrixf matR = WM.getJointTrans(/*JID_LEG_R_5 TODO*/0);
			static Vector3f startRFootPos = matR.p();

			posBall = WM.getBallGlobalPos();
            Timing::controlJointAngle(jact,
                                      0,//TODO JID_LEG_R_2,
                                      WM.lastPerception().joints().jointAng(/*JID_LEG_R_2 TODO*/0),
                                      ang2,
                                      time2);

			//jact->set(JID_LEG_R_2,1400);
			//LOG_PRINTF("kick","JID_LEG_R_2 = %f",jact->getDegree(JID_LEG_R_2));

			if(isFirstDoAction())
				startRFootPos = matR.p();
			//AngDeg canKickAng = acosDeg((WM.getJointTrans(JID_LEG_R_2).p()[2] -0.964 - 0.095) / 1.35);
			//if((matR.p() - startRFootPos).length() > 0.2)
			if(!isFirstDoAction())
			{
                Timing::controlJointAngle(jact,
                                          0,//JID_LEG_R_4,
                                          WM.lastPerception().joints().jointAng(0/*TODO JID_LEG_R_4*/),
                                      ang4,
                                      time4);
				//jact->set(JID_LEG_R_4,1000);
				//LOG_PRINTF("kick","JID_LEG_R_4 = %f,length = %f",jact->getDegree(JID_LEG_R_4),(matR.p() - startRFootPos).length());
			}
			LOG_PRINTF("kick","use right leg to kick,high = %f ",WM.getJointTrans(JID_LEG_R_2).p()[2]);
			//mTest_haveKick = true;
		}
		else
		{
            Timing::controlJointAngle(jact,
                                      0,//TODO JID_LEG_L_2,
                                      WM.lastPerception().joints().jointAng(0/*TODO JID_LEG_L_2*/),
                                      ang2,
                                      time2);
			//AngDeg canKickAng = acosDeg((WM.getJointTrans(JID_LEG_L_2).p()[2] -0.964 - 0.095) / 1.35);
			if(!isFirstDoAction())
			{
                Timing::controlJointAngle(jact,
                                          0,//TODO JID_LEG_L_4,
                                          WM.lastPerception().joints().jointAng(0/*JID_LEG_L_4*/),
                                      ang4,
                                      time4);
				//LOG_PRINTF("kick","JID_LEG_L_4 = %f",jact->getDegree(JID_LEG_L_4));
			}
			LOG_PRINTF("kick","use left leg to kick,high = %f",WM.getJointTrans(JID_LEG_R_2).p()[2]);
			//mTest_haveKick = true;
		}

		LOG_FLUSH;
		return mActionCache;
    }


	bool AccelerateFoot::isDone() const
	{
		if ( isTimeOut() ) return true;

		if(getRemainTime() < 0.84 ) return true;

		return false;

	}





} // namespace task
