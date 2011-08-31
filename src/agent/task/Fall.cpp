/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Fall.cpp 
 *
 ****************************************************************************/
#include "core/WorldModel.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"
#include "Fall.h"
namespace task {
	using namespace boost;
	using namespace action;
	using namespace serversetting;
	
	RightFall::RightFall(Task * primary):Task(-1, primary)
 	{	
	mTaskName = "testfall";
  	mDuration = FAT.calTaskTime(mTaskName);	
 	}
 
 
	boost::shared_ptr<action::Action> RightFall::perform()
	{
		if ( mStartTime < 0)
		{
			mStartTime =WM.getSimTime();
		}
	return FAT.controlPreferThan(mTaskName,"*");
	}

 	LeftFall::LeftFall(Task * primary):Task(-1, primary)
 	{	
		mTaskName = "testfall";
  		mDuration = FAT.calTaskTime(mTaskName);	
 	}
 

	boost::shared_ptr<action::Action> LeftFall::perform()
	{
		if ( mStartTime < 0)
		{
			mStartTime =WM.getSimTime();
		}
	return FAT.controlPreferThan(mTaskName,"*");
	}

	Fall::Fall():Task(-1,NULL)
	{
		mDir=UNKNOWN;			
	}

	Fall::Fall(Task * primary,Direction dir):Task(-1,primary)
	{	
		mDir=dir;
		if(dir==RIGHT) 
		{
			if(WM.isTouch(0)) mTaskName = "rightfall_pt_int_squat";	
			else mTaskName = "rightfall_pt_init";
		}
		if(dir==LEFT)
		{
			if(WM.isTouch(1)) mTaskName = "leftfall_pt_int_squat";	
			else mTaskName = "leftfall_pt_init";
		}
                if(dir ==  FRONT){
                    mTaskName = "frontfall";
                }
                if(dir == BACK){
                    mTaskName = "backfall";
                }
				
		mDuration=FAT.calTaskTime(mTaskName);
	}
	
	Fall::~Fall()
	{
	}

	boost::shared_ptr<action::Action> Fall::perform()
	{
    	if ( mStartTime < 0 ){
            // first perform
            mStartTime = WM.getSimTime();
        }
		updateSubTaskList();
			
			
		return  FAT.controlPreferThan(mTaskName,"*");
	}
	
	bool Fall::isDone() const
	{
		return isTimeOut();
	}
	
	
	void Fall::SetDirection(Direction dir )
	{
		mDir=dir;
	}
	
	KeepFall::KeepFall():Task(-1,NULL)
	{
		mDir=UNKNOWN;		
	}

	KeepFall::KeepFall(Task * primary,Direction dir):Task(-1,primary)
	{	
		mDir=dir;
		if(dir==RIGHT)mDuration=FAT.calTaskTime("rightfall_pt_2_6");
			else mDuration=FAT.calTaskTime("leftfall_pt_2_6");
	}
	
	KeepFall::~KeepFall()
	{
	}

	boost::shared_ptr<action::Action> KeepFall::perform()
	{
    
		updateSubTaskList();
		
			
		if(mDir==RIGHT)
		{
			
			return  FAT.controlPreferThan("rightfall_pt_2_6","*");
		}
		else 
		{
				
			return FAT.controlPreferThan("leftfall_pt_2_6","*");
		}
    	
		
		
	}
	
	bool KeepFall::isDone() const
	{
		return isTimeOut();
	}
	
	
	void KeepFall::SetDirection(Direction dir )
	{
		mDir=dir;
		if(dir==RIGHT)mDuration=FAT.calTaskTime("rightfall_pt_2_6");
			else mDuration=FAT.calTaskTime("leftfall_pt_2_6");
	}
}

