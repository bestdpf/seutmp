/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: JointAction.h,v 1.2 2007/03/13 07:34:10 xy Exp $
 *
 ****************************************************************************/

#ifndef ACTION_JOINT_ACTION_H
#define ACTION_JOINT_ACTION_H

#include "Action.h"
#include "math/Math.hpp"
#include<stdio.h>//////////////////////////////////// test


namespace action {

class JointAction : public Action
{
public:

	//TT, March, MMXI
	//[b] false for jid: 0,1 (camera)
	//    true for jid: 2 to 21
	JointAction(bool b=true);

	virtual ~JointAction(){};

	virtual std::string command() const;

	//TT: you need to know that "r" is turning speed of joint
	void set(unsigned int jid, math::AngDeg r)
	{
		if(jid>=2)
			mJointMap[jid]=math::deg2Rad(r);
	}

	void setForCamera(unsigned int jid, math::AngDeg r); //terry


	bool get(unsigned int jid, math::AngDeg& r) const
	{
		TJointMap::const_iterator iter = mJointMap.find(jid);
		if(iter != mJointMap.end())
		{
			r = iter->second;
			return true;
		}
		return false;
	}


	bool getDegree(unsigned int jid, math::AngDeg& r) const
	{
		if(get(jid, r))
		{
			r = math::rad2Deg(r);
			return true;
		}
		printf("JA::getDegree() false. jid= %d\n",jid);
		return false;
	}


	void clear()
	{
		mJointMap.clear();
	}


	//fill all joint with v
	//TT: it doesn't include jid 0 and 1
	void fill(float v);

	//TT, April, MMXI
	//for predicted perception
	//return true if it doesn't include head
	bool isBodyJointAction() const {
		return mIsBodyJointAction;
	}


private:

	// mapping from joint id to joint hinge object
	typedef std::map<unsigned int, float> TJointMap; //it's radian instead of angle-degree
	TJointMap mJointMap;

	bool mIsBodyJointAction;

};


} //end of namespace action

#endif // ACTION_JOINT_ACTION_H

