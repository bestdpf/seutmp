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
#include "BackLeg.h"

namespace task{

    using namespace std;
    using namespace math;
    using namespace serversetting;
	using namespace boost;

	
	DEFINE_STATIC_GRAPHIC_LOGGER(BackLeg)
	
    BackLeg::BackLeg(Task* primary)
        :Task(0.5, primary)
    {
		 
        BEGIN_ADD_STATIC_LOG_LAYER(BackLeg) 
        ADD_LOG_LAYER("kick"); 
        ADD_LOG_LAYER("goto"); 
        
        END_ADD_STATIC_LOG_LAYER(BackLeg)
        
    }

    shared_ptr<action::Action> BackLeg::perform()
    {
		Task::perform();
		
		Kick* p = dynamic_cast<Kick*>(mParentTask);
		if ( 0==p )
		{
			shared_ptr<Action> act;
			cout<<"backLeg change error"<<endl;
			return act;			
		}
		
		LOG_PRINTF("kick","do kick pose back leg");
		const static float minFootHeight = 0;//TODO SS.mFootHeight*0.5;
        const static float torsoMoveX = 0.35;
        mPoseParameter.hip.identity();
        mPoseParameter.hip.p().z() = 3.4578 - 1.00/2 - 0.55/2 + 0.25 - 0.55/2 - 0.3;
        mPoseParameter.hip.p().z() += 0.05;
        mPoseParameter.footL.identity(); mPoseParameter.footR.identity();
        mPoseParameter.footL.p().x() = -0.3900;		mPoseParameter.footR.p().x() = 0.3900;
        mPoseParameter.footL.p().z() = minFootHeight;	mPoseParameter.footR.p().z() = minFootHeight;
		Vector2f ballPos;
		ballPos = WM.getBallGlobalPos();
		if(p->getIsLeftLeg())	//use left leg to kick
		{
			mPoseParameter.hip.p().x() = torsoMoveX;
			mPoseParameter.hip.p().y() = 0.1;
			LOG_PRINTF("kick","ball 2 foot length = %f",(WM.getBallGlobalPos() - mPoseParameter.footR.p()).length());
			Vector3f foot;
			p->calKickFootLastPos(p->getTarget(),ballPos,foot);
			mPoseParameter.footL.p() = foot;
		}
		else
		{
			mPoseParameter.hip.p().x() = -torsoMoveX;
			mPoseParameter.hip.p().y() = 0.1;
			
			LOG_PRINTF("kick","ball 2 foot length = %f",(WM.getBallGlobalPos() - mPoseParameter.footR.p()).length());
			Vector3f foot;
			p->calKickFootLastPos(p->getTarget(),ballPos,foot);
			mPoseParameter.footR.p() = foot; 

			//mPoseParameter.footR.p().x() = 0.6;
            //mPoseParameter.footR.rotateLocalX(20);
            //mPoseParameter.footR.rotateLocalZ(angle);
		}
		p->generateDesiredJointAngles(mPoseParameter.hip,mPoseParameter.footL,mPoseParameter.footR,mDesiredJoints);
		mActionCache = controller::Timing::control(WM.lastPerception(), mDesiredJoints, getRemainTime());
       
		LOG_FLUSH;
		return mActionCache;
    }
	
	
	bool BackLeg::isDone() const
	{
		if ( isTimeOut() ) return true;
		
		return false;
		
	}
		
		
		
	
    
} // namespace task
