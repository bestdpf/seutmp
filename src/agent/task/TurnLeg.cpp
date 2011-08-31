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
#include "math/Matrix.hpp"
#include "TurnLeg.h"

namespace task{

    using namespace std;
    using namespace math;
    using namespace serversetting;
	using namespace boost;


	DEFINE_STATIC_GRAPHIC_LOGGER(TurnLeg)

    TurnLeg::TurnLeg(Task* primary)
        :Task(0.5, primary)
    {

        BEGIN_ADD_STATIC_LOG_LAYER(TurnLeg)
        ADD_LOG_LAYER("kick");
        ADD_LOG_LAYER("goto");

        END_ADD_STATIC_LOG_LAYER(TurnLeg)

    }

    shared_ptr<action::Action> TurnLeg::perform()
    {
		Task::perform();

		Kick* p = dynamic_cast<Kick*>(mParentTask);
		if ( 0==p )
		{
			shared_ptr<Action> act;
			cout<<"TurnLeg change error"<<endl;
			return act;
		}

        LOG_PRINTF("kick","do kick pose turn leg");
		const static float minFootHeight = 0;//TODO SS.mFootHeight*0.5;
        const static float torsoMoveX = 0.35;
        mPoseParameter.hip.identity();
        mPoseParameter.hip.p().z() = 3.4578 - 1.00/2 - 0.55/2 + 0.25 - 0.55/2 - 0.3;
        mPoseParameter.hip.p().z() += 0.05;
        mPoseParameter.footL.identity(); mPoseParameter.footR.identity();
        mPoseParameter.footL.p().x() = -0.3900;		mPoseParameter.footR.p().x() = 0.3900;
        mPoseParameter.footL.p().z() = minFootHeight;	mPoseParameter.footR.p().z() = minFootHeight;


		AngDeg angZ;
		AngDeg errAng;
		TransMatrixf matR,matL;
		matR = WM.getJointTrans(0);//TODO (JID_LEG_R_5);
        matL = WM.getJointTrans(0);//TODO (JID_LEG_L_5);
		Vector2f posBall;
    	posBall = WM.getBallGlobalPos();
		Vector2f foot2Ball;

		if(p->getIsLeftLeg())	//use left leg to kick
		{
			mPoseParameter.hip.p().x() = torsoMoveX;
			mPoseParameter.hip.p().y() = 0.1;
			Vector3f foot;
			p->calKickFootLastPos(p->getTarget(),posBall,foot);
			mPoseParameter.footL.p() = foot;

			foot2Ball = matL.p();
			foot2Ball = posBall - foot2Ball;
			AngDeg angFoot2Ball = atan2Deg(foot2Ball[1],foot2Ball[0]);

			angZ = atan2Deg(matR.n()[1],matR.n()[0]) + 90;
			AngDeg turnAngZ = angFoot2Ball - angZ;
			//AngDeg turnAngZ = angZ - angFoot2Ball;

			turnAngZ = normalizeAngle(turnAngZ);
			errAng = angZ + turnAngZ - (atan2Deg(matL.n()[1],matL.n()[0]) + 90);
			/*
			if(fabs(errAng) < 3)
			{
				delFirstElement();
				LOG_PRINTF("kick","del fisrst element");
			}
			*/
			TransMatrixf mat = WM.getMyOriginTrans();
			mat.transfer(foot);
			LOG_PRINTF("kick","errPos = %f,turnAngZ = %f,errAng = %f",(matL.p() - mat.p()).length(),turnAngZ,errAng);
			Vector3f temp = WM.getBallGlobalPos() - matL.p();
			LOG_LINE("kick",matL.p(),matL.p() +(Vector3f(temp[0],temp[1],0))*30,logger::Color::YELLOW);
			LOG_LINE("kick",matL.p(),matL.p() + Vector3f(10,0,0),logger::Color::BLUE);
			LOG_LINE("kick",matR.p(),matR.p() + Vector3f(cosDeg(angZ)*10,sinDeg(angZ)*10,0),logger::Color::RED);
			LOG_LINE("kick",matL.p(),matL.p() + Vector3f(cosDeg(angZ)*10,sinDeg(angZ)*10,0),logger::Color::RED);
			if(errAng > 0)
				turnAngZ += 10;
			else
				turnAngZ -= 10;
            mPoseParameter.footL.rotateLocalZ(turnAngZ);
			/*ofstream out("kick_t.txt");
			out<<(matL.p() - mat.p()).length()<<"  "<<errAng<<" "<<p->getIsLeftLeg()<<endl;
			out.close();*/

		}
		else
		{
			mPoseParameter.hip.p().x() = -torsoMoveX;
			mPoseParameter.hip.p().y() = 0.1;
            //mPoseParameter.footR.rotateLocalY(20);

			Vector3f foot;
			p->calKickFootLastPos(p->getTarget(),posBall,foot);
			mPoseParameter.footR.p() = foot;


			foot2Ball = matR.p();
			foot2Ball = posBall - foot2Ball;
			AngDeg angFoot2Ball = atan2Deg(foot2Ball[1],foot2Ball[0]);

			angZ = atan2Deg(matL.n()[1],matL.n()[0]) + 90;
			AngDeg turnAngZ = angFoot2Ball - angZ;

			turnAngZ = normalizeAngle(turnAngZ);
			errAng = angZ + turnAngZ - (atan2Deg(matR.n()[1],matR.n()[0]) + 90);
			/*
			if(fabs(errAng) < 3)
			{
				delFirstElement();
				LOG_PRINTF("kick","del fisrst element");
			}
			*/
			TransMatrixf mat = WM.getMyOriginTrans();
			mat.transfer(foot);
			LOG_PRINTF("kick","errPos = %f,turnAngZ = %f,errAng = %f",(matR.p() - mat.p()).length(),turnAngZ,errAng);
			Vector3f temp = WM.getBallGlobalPos() - matR.p();
			Vector3f temp2 = WM.getBallGlobalPos() - mat.p();
			LOG_BLUE_SPHERE("kick",mat.p(),0.2);
			LOG_LINE("kick",mat.p(),Vector3f(p->getTarget()[0],p->getTarget()[1],0),logger::Color::WHITE);
			LOG_RED_SPHERE("kick",Vector3f(p->getTarget()[0],p->getTarget()[1],0),0.2);
			LOG_PRINTF("kick","mParameter_kickTarget = %f,%f",p->getTarget()[0],p->getTarget()[1]);
			LOG_PRINTF("kick","length form foot to ball = %f",(WM.getBallGlobalPos() - matR.p()).length());
			LOG_LINE("kick",mat.p(),mat.p()+(Vector3f(temp2[0],temp2[1],0))*30,logger::Color::GREEN);
			LOG_LINE("kick",matR.p(),matR.p() +(Vector3f(temp[0],temp[1],0))*30,logger::Color::YELLOW);
			LOG_LINE("kick",matR.p(),matR.p() + Vector3f(10,0,0),logger::Color::BLUE);
			LOG_LINE("kick",matL.p(),matL.p() + Vector3f(cosDeg(angZ)*10,sinDeg(angZ)*10,0),logger::Color::RED);
			LOG_LINE("kick",matR.p(),matR.p() + Vector3f(cosDeg(angZ)*10,sinDeg(angZ)*10,0),logger::Color::RED);

			if(errAng > 0)
				turnAngZ += 10;
			else
				turnAngZ -= 10;

            mPoseParameter.footR.rotateLocalZ(turnAngZ);
			//mPoseParameter.footR.rotateLocalZ(30);


		}

		p->generateDesiredJointAngles(mPoseParameter.hip,mPoseParameter.footL,mPoseParameter.footR,mDesiredJoints);
		mActionCache = controller::Timing::control(WM.lastPerception(), mDesiredJoints, getRemainTime());



		//******************log***********************
		// below lines are all log
        // these should be commented out while complete the debug
        // translate the local position to global position
#if 0
        LOG_PRINTF("kick","remain time = %f",getRemainTime());
        stringstream ss;
        TransMatrixf myMat,tempMat;
        int supFoot = WM.getMySupportFoot();
        if (supFoot>0) myMat = WM.getJointTrans(JID_LEG_R_5);
        else myMat = WM.getJointTrans(JID_LEG_L_5);
        myMat.transfer(Vector3f(supFoot>0?-0.39:0.39,0,0));
        myMat.pos().z() = 0;
        ss<<"P: "<<myMat.pos();
        tempMat = myMat;
        LOG_YELLOW_BOX("kick",tempMat,Vector3f(0.2,0.2,0.2));
        tempMat.transfer(mPoseParameter.hip);
        LOG_RED_BOX("kick",tempMat,Vector3f(0.2,0.2,0.2));
        tempMat = myMat;
        tempMat.transfer(mPoseParameter.footL);
        LOG_RED_BOX("kick",tempMat,SS.getSize(JID_LEG_L_5));
        tempMat = myMat;
        tempMat.transfer(mPoseParameter.footR);
        LOG_RED_BOX("kick",tempMat,SS.getSize(JID_LEG_R_5));

        ss<<" L: "<<mPoseParameter.footL.pos()<<" Hip: "<<mPoseParameter.hip.pos()<<" R: "<<mPoseParameter.footR.pos();
        LOG_PRINT("kick",ss.str());
#endif

        Vector2f myPos; myPos = WM.getMyOrigin();
        Vector2f relPosBall = posBall - myPos;
		shared_ptr<JointAction> jact = shared_static_cast<JointAction>(mActionCache);
		//LOG_PRINTF("kick","JID from R1 -5 : %f,%f,%f,%f,%f",jact->getDegree(JID_LEG_R_1),jact->getDegree(JID_LEG_R_2)
		//		   ,jact->getDegree(JID_LEG_R_3),jact->getDegree(JID_LEG_R_4),jact->getDegree(JID_LEG_R_5));


		LOG_FLUSH;
		return mActionCache;
    }


	bool TurnLeg::isDone() const
	{
		if ( isTimeOut() ) return true;


		AngDeg angZ;
		AngDeg errAng;
		TransMatrixf matR,matL;
		matR = WM.getJointTrans(0);// TODO (JID_LEG_R_5);
		matL = WM.getJointTrans(0);//TODO (JID_LEG_L_5);
		Vector2f posBall;
    	posBall = WM.getBallGlobalPos();
		Vector2f foot2Ball;

		Vector3f foot;
		Kick* p = dynamic_cast<Kick*>(mParentTask);

		if ( 0==p )
		{
			shared_ptr<Action> act;
			cout<<"TurnLeg change error"<<endl;
			return act;
		}
		//p->calKickFootLastPos(p->getTarget(),posBall,foot);

		AngDeg angFoot2Ball;
		AngDeg turnAngZ;
		if(p->getIsLeftLeg())
		{
		//	mPoseParameter.footL.p() = foot;
			foot2Ball = matL.p();
			foot2Ball = posBall - foot2Ball;
			angFoot2Ball = atan2Deg(foot2Ball[1],foot2Ball[0]);

			angZ = atan2Deg(matR.n()[1],matR.n()[0]) + 90;
			turnAngZ = angFoot2Ball - angZ;
			turnAngZ = normalizeAngle(turnAngZ);
			errAng = angZ + turnAngZ - (atan2Deg(matL.n()[1],matL.n()[0]) + 90);
		}
		else
		{
		//	mPoseParameter.footR.p() = foot;
			foot2Ball = matR.p();
			foot2Ball = posBall - foot2Ball;
			angFoot2Ball = atan2Deg(foot2Ball[1],foot2Ball[0]);
			angZ = atan2Deg(matL.n()[1],matL.n()[0]) + 90;
			turnAngZ = angFoot2Ball - angZ;
			turnAngZ = normalizeAngle(turnAngZ);
			errAng = angZ + turnAngZ - (atan2Deg(matR.n()[1],matR.n()[0]) + 90);
		}




		static bool isDo = false;
		if(isDo)
		{
			isDo = false;
			return true;
		}
		if(fabs(errAng) < 3)
		{
			LOG_PRINTF("kick","del fisrst element");
			isDo = true;
			return false;
		}
		LOG_PRINTF("kick","(isDone) :errAng = %f",errAng);
		return false;
	}





} // namespace task
