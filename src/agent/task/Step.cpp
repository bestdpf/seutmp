/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Step.cpp 2139 2008-07-23 09:06:33Z xy $
 *
 ****************************************************************************/

#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "Step.h"
#include "SwingFoot.h"
#include "iostream"
namespace task{

using namespace std;
using namespace boost;
using namespace math;
using namespace robot::humanoid;
using namespace action;

Vector2f Step::mMaxSizeAcc(0.02f, 0.07f);
Vector2f Step::mMinSize(-0.03, -0.16);
Vector2f Step::mMaxSize(0.1, 0.16);

AngDeg Step::mMaxDirAcc = 50.0f;
AngDeg Step::mMinDir = 0.0f;
AngDeg Step::mMaxDir = 50.0f;
float Step::mStepTime = 0.4;

Step::Step( bool isLeft,
			const Vector2f& size, AngDeg dir,
			shared_ptr<const Step> preStep,
			float bodyHeight,
			Task* primary )
	:Task(-1, primary),
	 mIsLeft(isLeft)
{
	Vector2f preSize(0,0);
	Vector2f preSizeAcc(0,0);
	AngDeg preDir = 0;
	if ( 0!=preStep.get() ){
		preSize = preStep->size();
		preSizeAcc = preStep->sizeAcc();
		preDir = preStep->dir();
	}
	mSize = size;
	/////////////////////////////////////////////////////////////
	/// restrict the step size and direction

	//----------------------------------------------------------
	// restrict the turning angle
	AngDeg maxDirAcc = mMaxDirAcc;
	AngDeg maxDir = mIsLeft?mMaxDir:-mMinDir;
	AngDeg minDir = mIsLeft?mMinDir:-mMaxDir;

	AngDeg dirAcc = calClipAng( dir, preDir );
	dirAcc = clamp(dirAcc, -maxDirAcc, maxDirAcc);
	float maxsizelen = mMaxSize.length();
	float presizelen = preSize.length();
	dirAcc *= max(0.5f, 1-presizelen/maxsizelen );
	mDir = dirAcc + preDir;
	mDir = clamp(mDir, minDir, maxDir);
	Vector2f rotateMov;
	float halfFeetWidth = HUMANOID.getHalfFeetWidth();
	rotateMov.x() = (mIsLeft?1:-1) * sign(mDir)
		* halfFeetWidth * ( 1 - cosDeg(mDir) ) * 2;
	rotateMov.y() = (mIsLeft?-1:1) * halfFeetWidth * sinDeg(mDir) * 2;

	//----------------------------------------------------------
	// restirct the step size

	Vector2f maxSize, minSize;
	maxSize.x() = (mIsLeft?-mMinSize.x():mMaxSize.x());
	maxSize.y() = mMaxSize.y() * cosDeg(mDir);
	minSize.x() = (mIsLeft?-mMaxSize.y():mMinSize.y());
	minSize.y() = mMinSize.y() * cosDeg(mDir);
	
	if ( mSize.squareLength() > 1 ){
		// to avoid side walk by noise data
		mSize.normalize();
		mSize*=2;
	}

	mSize += rotateMov;
	mSizeAcc = mSize - preSize;
	mSizeAcc.x() = clamp( mSizeAcc.x(),
						  -mMaxSizeAcc.x(), mMaxSizeAcc.x());
	mSizeAcc.y() = clamp( mSizeAcc.y(),
						  -mMaxSizeAcc.y(), mMaxSizeAcc.y());
	mSizeAcc *= cosDeg(mDir);
	mSize = preSize + mSizeAcc;

	// restric walk side according to forward or backward
	float maxLen = maxSize.length();
	float len = mSize.length();
	bool slowDown = false;
	if ( len > maxLen ){
		mSize *= (maxLen/len);
	}
	else if( len < 0.03f){
		LOG_PRINTF("new","slow down");
		slowDown = true;
	}
	mSize.x() = clamp(mSize.x(), minSize.x(), maxSize.x());
	mSizeAcc = mSize - preSize;

	// create sequences of one step
	const float loft_foot_height = HUMANOID.getMinFootHeight();
	float swingHeight = slowDown?(loft_foot_height):(loft_foot_height*1.8f);

	TransMatrixf t0;
	t0.rotationZ(preDir);
	float feetx = (mIsLeft?1:-1)*HUMANOID.getHalfFeetWidth();
	t0.p().x() = preSize.x();
	t0.p().y() = preSize.y();
	t0.p().x() += feetx;
	t0.transfer(Vector3f(-feetx,0,0));
	TransMatrixf t1;
	t1.identity();
	t1.p().x() = -feetx;
	TransMatrixf t2 = t0;
	t2.inverseTransfer(t1);

	Vector2f startPoint;
	startPoint.x() = t2.p().x() + feetx;
	startPoint.y() = t2.p().y();
	//printf startPoint
	std::cout<<"start point ... "<<std::endl;
	std::cout<<startPoint.x()<<"\t"<<startPoint.y()<<std::endl;
	Vector2f swingVec = mSize - startPoint;
	float totalTime = 0.2;// allen change from 0.2 to 0.15 June 18
	float stepTime = 2 * ceil(WM.getAverageStepTime() / serversetting::sim_step) * serversetting::sim_step;
	int n = max(3, int(ceil(totalTime/stepTime)));
	swingVec /= n;
	AngDeg swingAng = (mDir+preDir) / n;
	float a = - (4*swingHeight/n/n);
	float b = 4*swingHeight/n;
	AngDeg rotateFoot = 0;
	if (mSize.y()>0.02){
		if ( stepTime < 0.05f ){
			rotateFoot = 10;
		}
		else{
			rotateFoot = 2.5f;
		}
	}

	for(int i=1; i<n+1; i++){
		float h = a*i*i + b*i;
		shared_ptr<Task> swingFoot
			( new SwingFoot(mIsLeft, startPoint + swingVec*i,
							h,
							-preDir + swingAng*i,
							rotateFoot,
							bodyHeight, stepTime, this ) );
		mSubTaskList.push_back(swingFoot);
	}

	LOG_FLUSH;
} //end of Step()


bool Step::isDone() const
{
	return Task::isDone();

	}

shared_ptr<Action> Step::perform()
{
	// try do sub task
	shared_ptr<Action> act = Task::perform();

	if ( 0==act.get() ){
		// there is no sub task
		shared_ptr<JointAction> jact( new JointAction );
		jact->fill(0);
		act = shared_static_cast<Action>(jact);
	}

	return act;
}


bool Step::isTerminable() const
{
	// if double support, the aciton can be breaked
	return mSubTaskList.size() < 2
		&& WM.isTouch(/*FRID_LEFT_FOOT TODO*/0) && WM.isTouch(/*FRID_RIGHT_FOOT*/1);
}


} //end of namespace task

