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

//Vector2f Step::mMaxSizeAcc(0.0135f, 0.045f);
//Vector2f Step::mMinSize(-0.027, -0.09);
//Vector2f Step::mMaxSize(0.027, 0.09);
Vector2f Step::mMaxSize=Vector2f(0.05,0.16);
Vector2f Step::mMinSize=mMaxSize*(-1.0f);
Vector2f Step::mMaxSizeAcc=mMaxSize*0.5f;
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
	//calc the rotate vector
	Vector2f rotateMov;
	float halfFeetWidth = HUMANOID.getHalfFeetWidth();
	rotateMov.x() = (mIsLeft?1:-1) * sign(mDir)
		* halfFeetWidth * ( 1 - cosDeg(mDir) ) * 2;
	rotateMov.y() = (mIsLeft?-1:1) * halfFeetWidth * sinDeg(mDir) * 2;
	///test walk code by dpf, keep it for use of future!!! don't delete it!
	/*
	math::TransMatrixf tmp;

	//if i am left, walk Coordinate is right foot's trans
	math::TransMatrixf leftFootRelTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::L_FOOT);
	math::TransMatrixf rightFootRelTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::R_FOOT);
	math::TransMatrixf walkCoordinateToRelTrans=(mIsLeft?rightFootRelTrans:leftFootRelTrans);//trans from walk coordinate to rel coordinate
	math::TransMatrixf moveFootTrans=(mIsLeft?leftFootRelTrans:rightFootRelTrans);
	*/
	/*
	//suppose the bodydir x is always -10 degree
	math::TransMatrixf tmp;
	math::TransMatrixf leftFootLocalTrans=WM.getBoneLocalTrans(robot::humanoid::Humanoid::L_FOOT);
	math::TransMatrixf rightFootLocalTrans=WM.getBoneLocalTrans(robot::humanoid::Humanoid::R_FOOT);
	math::TransMatrixf leanRelToRelTrans;
	leanRelToRelTrans.rotationX(-15);
	tmp=leanRelToRelTrans;
	math::TransMatrixf leftFootRelTrans=tmp.transfer(leftFootLocalTrans);
	tmp=leanRelToRelTrans;
	math::TransMatrixf rightFootRelTrans=tmp.transfer(rightFootLocalTrans);
	math::TransMatrixf walkCoordinateToRelTrans=(mIsLeft?rightFootRelTrans:leftFootRelTrans);//trans from walk coordinate to rel coordinate
	math::TransMatrixf moveFootTrans=(mIsLeft?leftFootRelTrans:rightFootRelTrans);
	
	if(mIsLeft){//right foot support, so the orgin should minuse a halfFeetWidth
	  walkCoordinateToRelTrans.p().x()-=HUMANOID.getHalfFeetWidth();
	}
	else{
	  walkCoordinateToRelTrans.p().x()+=HUMANOID.getHalfFeetWidth();
	}
	tmp=walkCoordinateToRelTrans;
	moveFootTrans=tmp.inverseTransfer(moveFootTrans);
	AngDeg footNowDir=moveFootTrans.rotatedAngZ();
	mSize=*(Vector2f*)(walkCoordinateToRelTrans.inverseTransform(Vector3f(mSize.x(),mSize.y(),0.0f)).get());
	*/
	//----------------------------------------------------------
	// restirct the step size

	Vector2f maxSize, minSize;
	maxSize.x() = mMaxSize.x();//(mIsLeft?-mMinSize.x():mMaxSize.x());
	maxSize.y() = mMaxSize.y() * cosDeg(mDir);
	minSize.x() = mMinSize.x();//(mIsLeft?-mMaxSize.x():mMinSize.y());
	minSize.y() = mMinSize.y() * cosDeg(mDir);
	cout<<"maxSize: "<<maxSize<<endl;
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
	mSize.y() = clamp(mSize.y(),minSize.y(),maxSize.y());
	mSizeAcc = mSize - preSize;

	// create sequences of one step
	const float loft_foot_height = HUMANOID.getMinFootHeight();
	float swingHeight = slowDown?(loft_foot_height):(loft_foot_height*1.8f);

	//calc startPoint, see details in our documents about walk, rewritted dy dpf
	
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
	//Vector2f swingVec = mSize - startPoint;
	
	/*
	Vector2f startPoint(0.0f,0.0f);
	TransMatrixf leftFootLocalTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::L_FOOT);
	TransMatrixf rightFootLocalTrans=WM.getBoneRelTrans(robot::humanoid::Humanoid::R_FOOT);
	//dpf, rel pos is relative to torso, walk rel is the coordinate systems describled in our documents
	Vector2f leftFootRel2D=*(Vector2f*)(leftFootLocalTrans.p().get());
	Vector2f rightFootRel2D=*(Vector2f*)(rightFootLocalTrans.p().get());
	AngDeg leftFootAng=leftFootLocalTrans.rotatedAngZ();
	AngDeg rightFootAng=rightFootLocalTrans.rotatedAngZ();
	//walk rel coordinate's origin pos in rel pos
	//float halfFeetWidth=Humanoid.getHalfFeetWidth();
	Vector2f pos1=leftFootRel2D+Vector2f(halfFeetWidth*cosDeg(leftFootAng),halfFeetWidth*sinDeg(leftFootAng));
	Vector2f pos2=rightFootRel2D-Vector2f(halfFeetWidth*cosDeg(rightFootAng),halfFeetWidth*sinDeg(rightFootAng));
	//Vector2f walkRelToRel2D=mIsLeft?rightFootRel2D:leftFootRel2D;//walk rel coordinate's origin pos2D
	//float feetx = (mIsLeft?1:-1)*HUMANOID.getHalfFeetWidth();
	//startPoint in rel pos2D (ie. body rel pos2D)
	//Vector2f startPointRel2D=(mIsLeft?leftFootRel2D:rightFootRel2D) + Vector2f(feetx,0);
	startPoint=(pos1-pos2)*(mIsLeft?1.0:-1.0f);
	*/
	/*
	//feedback test by dpf
	AngDeg leanAng=WM.getBoneTrans(robot::humanoid::Humanoid::TORSO).rotatedAngX();
        //danamic step size, test by dpf
	float addStepY=(-sinDeg(leanAng)*0.115-0.03)*0.25f;
	cout<<"addStepY: "<<addStepY<<endl;
	mSize.y()+=addStepY;
	*/
	/// test src of dpf for walk, keep it !!!
	/*
	if(mIsLeft){
	  mSize.x()+=-HUMANOID.getHalfFeetWidth();
	}
	else{
	  mSize.x()+=HUMANOID.getHalfFeetWidth();
	}
	//rel pos startPoint
	Vector2f startPoint=*(math::Vector2f*)(mIsLeft?leftFootRelTrans.p():rightFootRelTrans.p()).get();
	// from rel to walk rel
	startPoint=*(math::Vector2f*)(walkCoordinateToRelTrans.inverseTransform(Vector3f(startPoint.x(),startPoint.y(),0.0f))).get();
	*/
	Vector2f swingVec = mSize -startPoint;
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
	/*
	//dpf test cout
	cout<<(mIsLeft?"left":"right")<<endl;
	cout<<"mSize: "<<mSize<<endl;
	cout<<"small swingVec"<<swingVec<<endl;
	cout<<"swingHeight"<<swingHeight<<endl;
	*/
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

