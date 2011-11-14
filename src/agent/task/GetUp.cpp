/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "GetUp.h"
#include "core/WorldModel.h"
#include "controller/FixedAngleTrace.h"
#include "configuration/Configuration.h"

namespace task {

    using namespace std;
using namespace serversetting;

DEFINE_STATIC_GRAPHIC_LOGGER(PushUpDirectly)
DEFINE_STATIC_GRAPHIC_LOGGER(GetUpFromLie)
DEFINE_STATIC_GRAPHIC_LOGGER(GetUpFromDive)

PushUpDirectly::PushUpDirectly(Task* primary)
	:Task(-1, primary)
{
	/*BEGIN_ADD_STATIC_LOG_LAYER(PushUpDirectly)
        ADD_LOG_LAYER("pushUpDirectly");
    END_ADD_STATIC_LOG_LAYER(PushUpDirectly)*/

	if ( isNotReady() )mTaskName = "push-upNotReady0.0";
	else if ( WM.getJointTrans(0/*JID_LEG_L_5 TODO*/).p().z() > 2.0f )
		mTaskName = "LRRolled";
	else mTaskName = "push-upPrepared1.0";

	mDuration = FAT.calTaskTime( mTaskName);

	bufferTime = 0.0f;
}

bool PushUpDirectly::shouldDo(int currentState, int possibleState)
{
    return false;
    return 2 == currentState;

	float leftLegHeight = 0;//TODO WM.getJointTrans(JID_LEG_L_4).p().z();
	float rightLegHeight = 0;//TODO WM.getJointTrans(JID_LEG_R_4).p().z();
	float leftArmHeight = 0; //TODO WM.getJointTrans(JID_ARM_L_4).p().z();
	float rightArmHeight = 0;//TODO WM.getJointTrans(JID_ARM_R_4).p().z();

	if ( FAT.isPoseReached( "push-upPrepared0.1", AngDeg(3.5f)) )		
	{
		if ( abs( leftLegHeight - 0.70f ) < ( 0.12f ) && abs( rightLegHeight - 0.70f ) < ( 0.12f )
			&& abs( leftArmHeight - 0.72f ) < ( 0.12f ) && abs( rightArmHeight - 0.72f ) < ( 0.12f )
			&& ( abs( leftLegHeight - rightLegHeight ) < 0.02f )
			&& ( abs( leftArmHeight - rightArmHeight ) < 0.02f ) )	
		{
			if ( currentState == possibleState && currentState == 2 )		
				return true;
		}
	}
	return false;
}

bool PushUpDirectly::isNotReady()
{
	return !FAT.isPoseReached("push-upPrepared0.1",30.0f);
}

boost::shared_ptr<action::Action> PushUpDirectly::perform()
{
	if ( mStartTime < 0 )
	{
		mStartTime = WM.getSimTime();
	}

	if ( mTaskName == "push-upPrepared1.0" )
	{
		Vector3f mGyroRate = WM.getMyGyroRate();
		Vector3f posMe = WM.getMyGlobalPos();
		Vector3f mOrigin = WM.getMyOrigin();

		LOG_PRINTF("pushUpDirectly","mDuration %f, mDuration-remainTime %f", mDuration, mDuration-getRemainTime());
		LOG_PRINTF("pushUpDirectly","GyroRate %f, %f, %f",mGyroRate.x(), mGyroRate.y(), mGyroRate.z());
		LOG_PRINTF("pushUpDirectly","posMe - origin %f, %f, %f",(posMe-mOrigin).x(), (posMe-mOrigin).y(), (posMe-mOrigin).z());


		if ( ( mDuration - getRemainTime() ) > 0.07f && ( mDuration - getRemainTime() ) < 0.11f )
		{
			if (!( abs(mGyroRate.x()) > 50.0f && abs(mGyroRate.y()) <10.0f && abs(mGyroRate.z()) < 10.0f ))	//涓嶆垚鍔�
			{
				LOG_PRINT("pushUpDirectly","cancel the push-up");
				mDuration = 0.0f;			
				LOG_FLUSH
				return FAT.controlPreferThan("push-upPrepared0.1","*");
			}
		}
        float maxBufferTime = 10;
        if ( WM.isCloseToGoal() ){
            maxBufferTime = 2;
        }

		if ( getRemainTime() < 0.05f && bufferTime < maxBufferTime )
		{
			if ( WM.getMyGlobalPos().z() < 2.5f )mDuration = 0.0f;
			else
			{
				Vector2f footForce(100,100);
                if ( WM.isDoubleSupport() )
                    footForce = Vector2f(0,0);//TODO WM.lastPerception().forceResistance().feedBack(FRID_LEFT_FOOT).force + WM.lastPerception().forceResistance().feedBack(FRID_RIGHT_FOOT).force;
				if ( footForce.squareLength() > 1
                     || WM.getMyGyroRate().squareLength() > 0.003f )
				{
                    float deltaTime = WM.getLostSimTime() + sim_step;
                    mDuration += deltaTime;
                    bufferTime += deltaTime;
				}
			}
		}
	}

	LOG_FLUSH
	return FAT.controlPreferThan(mTaskName,"*");
}


PushUpPre::PushUpPre(int currentState, int possibleState, Task* primary):Task(-1, primary)
{
	if ( abs ((WM.lastPerception().joints().jointAng(/*JID_ARM_L_1 TODO*/0))
			- 165.0f ) < 20.0f )		
	{
		taskName = "push-upPrepared0.0";
		mTerminable = false;
	}
	else
	{
		if ( WM.getMyGlobalPos().z() < 0.8f
             && abs(WM.lastPerception().joints().jointAng(0/* TODO JID_ARM_L_4*/) - 115.0f) > 30.0f ){
            taskName = "push-upNotReady0.0";
            mTerminable = false;
        }
        else{
            taskName = "push-upPrepared0.1";
            mTerminable = true;
        }

	}
	mDuration = FAT.calTaskTime( taskName );
}

bool PushUpPre::shouldDo(int currentState, int possibleState)
{
	if ( currentState == 3 )return true;
	if ( currentState == 2 && !PushUpDirectly::shouldDo(currentState, possibleState) )return true;

	return false;
}

bool PushUpPre::isNotReady()
{
	float leftArmHeight = 0;//TODO WM.getJointTrans(JID_ARM_L_4).p().z();
	float rightArmHeight = 0;//TODO WM.getJointTrans(JID_ARM_R_4).p().z();

	bool isPushDirectNotReady(true);
    //TODO
		// ( !( ( abs(WM.lastPerception().joints().jointAng(JID_ARM_R_4) - (AngDeg) (135.0) ) <= 30.0 )
			// && ( abs(WM.lastPerception().joints().jointAng(JID_ARM_L_4) - (AngDeg) (135.0) ) <= 30.0 )
			// && ( abs(WM.lastPerception().joints().jointAng(JID_ARM_R_1) ) <= 30.0 )
			// && ( abs(WM.lastPerception().joints().jointAng(JID_ARM_L_1) ) <= 30.0 ) ) );

	return isPushDirectNotReady && ( ( leftArmHeight < 0.9f ) || ( rightArmHeight < 0.9f ) );
}

boost::shared_ptr<action::Action> PushUpPre::perform()
{
	if ( mStartTime < 0 )
	{
		mStartTime = WM.getSimTime();
	}

	return FAT.controlPreferThan(taskName, "*");
}

MantodeaUpDirectly::MantodeaUpDirectly(Task* primary)
	:Task(-1, primary)
{
	mTaskName = "mantodea_lied2";
	mDuration = FAT.calTaskTime(mTaskName);

	bufferTime = 0.0f;
}

bool MantodeaUpDirectly::shouldDo(int currentState, int possibleState)
{
    return false;
    return 0 == currentState;

    if ( currentState != 0 ) return false;

	float leftLegHeight = 0;//TODO WM.getJointTrans(JID_LEG_L_4).p().z();
	float rightLegHeight = 0;//TODO WM.getJointTrans(JID_LEG_R_4).p().z();
	float leftArmHeight = 0;//TODO WM.getJointTrans(JID_ARM_L_4).p().z();
	float rightArmHeight = 0;//TODO WM.getJointTrans(JID_ARM_R_4).p().z();

	if ( abs( leftLegHeight - 0.80f ) < ( 0.06f ) && abs( rightLegHeight - 0.80f ) < ( 0.06f )
			&& abs( leftArmHeight - 0.92f ) < ( 0.06f ) && abs( rightArmHeight - 0.92f ) < ( 0.06f )
			&& ( abs( leftLegHeight - rightLegHeight ) < 0.04f )
			&& ( abs( leftArmHeight - rightArmHeight ) < 0.04f ) )	
	{
		if ( FAT.isPoseReached( "mantodea_lied1", 3.5f ) )
		{
			if ( currentState == possibleState )		
			{
				return true;
			}
		}
	}
	return false;
}

boost::shared_ptr<action::Action> MantodeaUpDirectly::perform()
{
	if ( mStartTime < 0 )
	{
		mStartTime = WM.getSimTime();
	}

	if ( mTaskName == "mantodea_lied2" )
	{
		Vector3f mGyroRate = WM.getMyGyroRate();

		if ( ( mDuration - getRemainTime() ) > 0.11f && ( mDuration - getRemainTime() ) < 0.17f )
		{
			if (!( abs(mGyroRate.x()) > 100.0f && abs(mGyroRate.y()) < 26.0f && abs(mGyroRate.z()) < 26.0f ))	//涓嶆垚鍔�
			{
				mDuration = 0.0f;			
				return FAT.controlPreferThan("mantodea_lied1","*");
			}
		}
		float maxBufferTime = 10;
        if ( WM.isCloseToGoal() ){
            maxBufferTime = 2;
        }

		if ( getRemainTime() < 0.05f && bufferTime < maxBufferTime )
		{
			if ( WM.getMyGlobalPos().z() < 2.5f )mDuration = 0.0f;
			else
			{
				Vector2f footForce(100,100);
                if ( WM.isDoubleSupport() )
                    footForce = Vector2f(0,0);//TODO WM.lastPerception().forceResistance().feedBack(FRID_LEFT_FOOT).force
                //+ WM.lastPerception().forceResistance().feedBack(FRID_RIGHT_FOOT).force;
				if ( footForce.squareLength() > 1
                    || WM.getMyGyroRate().squareLength() > 0.003f )
				{
                    float deltaTime = WM.getLostSimTime() + sim_step;
					mDuration += deltaTime;
					bufferTime += deltaTime;
				}
			}
		}
	}

	return FAT.controlPreferThan(mTaskName, "*");
}

MantodeaUpPre::MantodeaUpPre(int currentState, int possibleState, Task* primary)
	:Task(-1, primary)
{
	if ( FAT.isPoseReached("push-upPrepared0.1",30.0f) )
	{
		taskName = "mantodea_lied0";	
		mTerminable = false;
	}
	else
	{
		mTerminable = true;
		taskName = "mantodea_lied1";
	}

	mDuration = FAT.calTaskTime(taskName);
}

bool MantodeaUpPre::shouldDo(int currentState, int possibleState)
{
	if ( currentState == 1 ) return true;
	if ( currentState == 0 && !MantodeaUpDirectly::shouldDo(currentState, possibleState) ) return true;

	return false;
}

boost::shared_ptr<action::Action> MantodeaUpPre::perform()
{
	if ( mStartTime < 0 )
	{
		mStartTime = WM.getSimTime();
	}
	return FAT.controlPreferThan(taskName, "*");
}

Roll::Roll(Task* primary):Task(-1, primary)
{
	mDuration = FAT.calTaskTime("LRRolled");
}

bool Roll::shouldDo(int currentState, int possibleState)
{
	return currentState == 4;
}

boost::shared_ptr<action::Action> Roll::perform()
{
	if ( mStartTime < 0 )
	{
		mStartTime = WM.getSimTime();
	}
	return FAT.controlPreferThan("LRRolled", "*");
}

ShakeBody::ShakeBody(Task* primary):Task(-1, primary)
{
	mDuration = FAT.calTaskTime("wriggle");
}

boost::shared_ptr<action::Action> ShakeBody::perform()
{
	if ( mStartTime < 0 )
	{
		mStartTime = WM.getSimTime();
	}
	return FAT.controlPreferThan("wriggle", "*");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//			Codes below: Only for test
//								7.1.   15:29
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
GetUpFromLie::GetUpFromLie(Task * primary):Task(-1,primary)
{
	/*BEGIN_ADD_STATIC_LOG_LAYER(GetUpFromLie)
    ADD_LOG_LAYER("getUpFromLie");
    END_ADD_STATIC_LOG_LAYER(GetUpFromLie)*/
	static int changeSteps_houqi = 0; //TT change "float" to "int"

	if( FAT.isPoseReached("houqi_lv_init",1.0f) )
	{
		LOG_PRINT("getUpFromLie","posReached");
		mTaskName = "houqi_lv_start";
		mTerminable = false;
		changeSteps_houqi = 0;
	}

	else
	{
		if(changeSteps_houqi > 2)
		{
			mTaskName = "houqi_lv_init_2";
			changeSteps_houqi = 0;
			mTerminable = false;
		}

		else
		{
			mTaskName = "houqi_lv_init";
			mTerminable = false;
			changeSteps_houqi += 1;
			LOG_PRINTF("getUpFromLie", "change steps is %f", changeSteps_houqi);
			//cout<<"changeSteps_houqi is :"<<changeSteps_houqi<<endl; //TT: always 1-2-3
		}
	}

	mDuration = FAT.calTaskTime(mTaskName);
	LOG_FLUSH;
}





bool GetUpFromLie::isDoneByAng() const
{
	return (abs(WM.getMyBodyAng().x())<5&&abs(WM.getMyBodyAng().y())<5);
}

bool GetUpFromLie::isTerminable() const
{
	if(mTaskName == "houqi_lv_init")
		{
		 if( FAT.isPoseReached("houqi_lv_init", 1.0f))
		 {
			LOG_PRINT("getUpFromLie", "init pose terminalble");
			LOG_FLUSH;
			 return true;
		 }
		 return false;
		}
	return mTerminable;

}

bool GetUpFromLie::isGetUpFailed() const
{
	if(!isDoneByTime())		return false;

	if(isDoneByAng())		return false;

	return	true;
}

boost::shared_ptr<action::Action> GetUpFromLie::perform()
{
	LOG_PRINTF("getUpFromLie","perform");
	LOG_PRINTF("getUpFromLie","%s",mTaskName.c_str());
	LOG_PRINT("getUpFromLie","current pos is "+FAT.getCurrentPose());
	LOG_FLUSH;
	if ( mStartTime < 0)
	{
		mStartTime =WM.getSimTime();
	}
	return FAT.controlPreferThan(mTaskName,"*");
}


GetUpFromDive::GetUpFromDive(Task * primary):Task(-1,primary)
{
	/*BEGIN_ADD_STATIC_LOG_LAYER(GetUpFromDive)
        ADD_LOG_LAYER("getUpFromDive");
    END_ADD_STATIC_LOG_LAYER(GetUpFromDive)*/

	static float changeSteps_qianqi = 0;
	bool isFallDive = FAT.isPoseReached("pituipq",10.0f);
	if(isFallDive)
	{
		mTaskName = "qianqi_lv_init_fall_dive";
		mTerminable = false;
		changeSteps_qianqi = 0;
 	}
	else
	{
		if (FAT.isPoseReached("qianqi_lv_init",7.0f)) //30.0f鍊煎緱鍟嗘Ψ
		{
			mTaskName = "qianqi_lv_start_init";
			mTerminable = false;
			changeSteps_qianqi = 0;
		}
	else
		{


			if(changeSteps_qianqi > 2)
			{
				mTaskName = "pushup_lv_init";//I will change a Pose if I
										//	can`t change Pose to qianqi_lv_init	in 1.2 sec
				mTerminable = false;
				changeSteps_qianqi = 0;
			}
			else
			{
				mTaskName = "qianqi_lv_init";
				changeSteps_qianqi += 1;
				//cout<<"changSteps_qianqi is "<<changeSteps_qianqi<<endl;
			}

		}
	}

	mDuration = FAT.calTaskTime(mTaskName);
	LOG_PRINTF("getUpFromDive","changeSteps_qianqi is : %.3f",changeSteps_qianqi);
	//cout<<"changeSteps_qianqi is"<<changeSteps_qianqi<<endl;
	LOG_FLUSH;
}


boost::shared_ptr<action::Action> GetUpFromDive::perform()
{
	LOG_PRINTF("getUpFromDive","perform");
	LOG_PRINTF("getUpFromDive","current pose is:%s",FAT.getCurrentPose().c_str());
	LOG_FLUSH;

	if ( mStartTime < 0)
	{
		mStartTime =WM.getSimTime();
	}
	return FAT.controlPreferThan(mTaskName,"*");
}

bool GetUpFromDive::isDoneByAng() const
{
	return (abs(WM.getMyBodyAng().x())<5&&abs(WM.getMyBodyAng().y())<5);
}

bool GetUpFromDive::isTerminable() const
{
	if("qianqi_lv_init" == mTaskName)
		return FAT.isPoseReached("qianqi_lv_init", 7.0f);
	else return mTerminable;

}

bool GetUpFromDive::isGetUpFailed() const
{
	if(!isDoneByTime())		return false;

	if(isDoneByAng())		return false;

	return	true;
}






LeftFallToLie::LeftFallToLie(Task * primary):Task(-1, primary)
{
// 	cerr<<"LeftFallToLie"<<endl;
	mTaskName = "LFTL";
	mTerminable = false;
	mDuration = FAT.calTaskTime(mTaskName);
}


bool LeftFallToLie::isDoneByAng() const
{
	return (WM.getMyBodyAng().x()>70&&abs(WM.getMyBodyAng().y())<10);
}



boost::shared_ptr<action::Action> LeftFallToLie::perform()
{
	if ( mStartTime < 0)
	{
		mStartTime =WM.getSimTime();
	}
	return FAT.controlPreferThan(mTaskName,"*");
}






RightFallToLie::RightFallToLie(Task * primary):Task(-1, primary)
{
// 	cerr<<"RightFallToLie"<<endl;
	mTaskName = "RFTL";
	mTerminable = false;
	mDuration = FAT.calTaskTime(mTaskName);
}


bool RightFallToLie::isDoneByAng() const
{
	return (WM.getMyBodyAng().x()>70&&abs(WM.getMyBodyAng().y())<10);
}



boost::shared_ptr<action::Action> RightFallToLie::perform()
{
	if ( mStartTime < 0)
	{
		mStartTime =WM.getSimTime();
	}
	return FAT.controlPreferThan(mTaskName,"*");
}


///////////


LeftFallToDive::LeftFallToDive(Task * primary):Task(-1, primary)
{
// 	cerr<<"LeftFallToDive"<<endl;
	mTaskName = "LFTLP";
	mTerminable = false;
	mDuration = FAT.calTaskTime(mTaskName);
}


bool LeftFallToDive::isDoneByAng() const
{
	return (WM.getMyBodyAng().x()>70&&abs(WM.getMyBodyAng().y())<10);
}



boost::shared_ptr<action::Action> LeftFallToDive::perform()
{
	if ( mStartTime < 0)
	{
		mStartTime =WM.getSimTime();
	}
	return FAT.controlPreferThan(mTaskName,"*");
}






RightFallToDive::RightFallToDive(Task * primary):Task(-1, primary)
{
// 	cerr<<"RightFallToDive"<<endl;
	mTaskName = "RFTLP";
	mTerminable = false;
	mDuration = FAT.calTaskTime(mTaskName);
}


bool RightFallToDive::isDoneByAng() const
{
	return (WM.getMyBodyAng().x()>70&&abs(WM.getMyBodyAng().y())<10);
}



boost::shared_ptr<action::Action> RightFallToDive::perform()
{
	if ( mStartTime < 0)
	{
		mStartTime =WM.getSimTime();
	}
	return FAT.controlPreferThan(mTaskName,"*");
}










}		//namespace


