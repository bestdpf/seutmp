/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: Walk.cpp 2753 2009-04-02 07:25:49Z zyj $
 *
 ****************************************************************************/

#include <list>
#include <boost/bind.hpp>
#include "AStar.hpp"
#include "core/WorldModel.h"
#include "configuration/Configuration.h"
#include "MoveFoot.h"
#include "Step.h"
#include "Walk.h"
#include "soccer/Player.h"																	////terrymimi

namespace task{
    using namespace std;
    using namespace boost;
    using namespace math;
    using namespace serversetting;
    using namespace action;
	using namespace soccer;													///terrymimi

    float Walk::mWalkHeight = 0.35f;
	int Walk::mForDeadBall = 0;

    DEFINE_STATIC_GRAPHIC_LOGGER(Walk)

Walk::Walk(const Vector2f& target, AngDeg direction, bool avoidBall, bool is4Kick,Vector3f threshold, int isForKick, bool isFastWalking,bool yuandi, Task* primary)
:Task(-1, primary), // -1: as soon as possible
	mTarget(target),
	mDirection(direction),
	mSizeErrorThreshold(threshold[0],threshold[1]),						/////terrymimi cancel it
	mDirErrorThreshold(threshold[2]),
	mIsWalking(isFastWalking),
	mAvoidBall(avoidBall),
	mIs4Kick(is4Kick),
	mForKick(isForKick),
	Yuandi(yuandi)
{
	/*BEGIN_ADD_STATIC_LOG_LAYER(Walk)
		ADD_LOG_LAYER("newWalk")
		ADD_LOG_LAYER("newStep")
		ADD_LOG_LAYER("planPath")
		ADD_LOG_LAYER("blocks")
		ADD_LOG_LAYER("terrymimi")
		ADD_LOG_LAYER("terrymimi-adjust")
		ADD_LOG_LAYER("edition6")
		ADD_LOG_LAYER("graz")
		ADD_LOG_LAYER("specialDribble")
	END_ADD_STATIC_LOG_LAYER(Walk)*/

	mPreSize.zero();
	mShouldStop=false;
}

bool Walk::isDone() const
{
	if (!mIsWalking) return false;

	return mSubTaskList.empty();
}

bool Walk::revise( shared_ptr<const Task> rt )
{
	shared_ptr<const Walk> wrt =
		shared_dynamic_cast<const Walk>(rt);
	if ( NULL != wrt.get() ){
		// accept another Walk
		mTarget = wrt->mTarget;
		mDirection = wrt->mDirection;
		mSizeErrorThreshold = wrt->mSizeErrorThreshold;
		mDirErrorThreshold = wrt->mDirErrorThreshold;
		mAvoidBall = wrt->mAvoidBall;
		return true;
	}

	return false;
}


void Walk::updateSubTaskList()
{
	bool isLeft = true;
	AngDeg preDir = 0;
	bool sideWalk = false;

	//Task
	Task::updateSubTaskList();

	//if it's too early to calculate next step, then return
	if( false==isSubTaskOfAllLessThanTwo() )
		return;

	shared_ptr<const Step> cStep;
	if( !mSubTaskList.empty() )
	{
		cStep = shared_dynamic_cast<const Step>( mSubTaskList.front() ); //get last step
		if( 0 != cStep.get() )
		{
			isLeft = !cStep->isLeft(); //change foot
			preDir  = cStep->dir();
			mPreSize = cStep->size();
		}
	}

	//TT, for Walk and WalkRel
	if( mShouldStop && mPreSize.length()<0.01f )
		return;


	//====================================================================
	// create the sequence tasks for start walking
	mIsWalking = true;
	//if(mShouldStop) mIsWalking=false;/////////////////////////////////////////////////////

	// plan the path
	planPath();

	Vector2f size = mPlanTarget - WM.getMyOrigin2D();
	if(mShouldStop)/////////////////////////////////////////////////////
		size=Vector2f(0,0);
	AngDeg aiming = mPlanDir;
	AngDeg currentDir = WM.getMyBodyDirection();
	AngDeg dir = calClipAng( aiming, currentDir );
	if(mShouldStop)////////////////////////////////////////////////////
		dir=0;
	Vector2f polar = xyz2pol(size);
	bool isTurning = false;

	//if( fabs(calClipAng(polar[1], aiming)) / polar[0] < Step::getMaxDir() / Step::getMaxSize().length() ) //TT rewrite
	if( fabs(calClipAng(polar[1],aiming)) * Step::getMaxSize().length() < Step::getMaxDir() * polar[0] )
	{
		// I am far from the target
		// redirect the aim to the target
		aiming = polar[1];
		dir = calClipAng( aiming, currentDir );
			shouldback = true;
		// back/forward
		if ( fabs(dir) > 90 ){
			if ( fabs(dir) > 120 || (isLeft && dir<0) || (!isLeft && dir>0) ){
				dir = normalizeAngle( dir + 180 );
						shouldback = false;
			}
		}

		if ( fabs(dir) > Step::getMaxSize().angle()||Yuandi ){ ////防止走动时转角过大
			// turning the direction firstly
			polar[0] = 0;		// 原地转动
			isTurning = true;
		}
	}

	polar[1] = calClipAng( polar[1], currentDir-90 );
	size = pol2xyz(polar);

	/////terrymimi
	if(abs(size.x()/size.y()) > 3&&size.length() < 3){
		sideWalk = true;
	}

	size *= 0.5f; // NOTE: the real step size is double 'size'


	//=================================TT=====================================
	const Vector3f& bodyAcc=WM.getMyAcc();

	if(false==WM.getLocalWalkSign())			//global walk
	{
		if( fabs(size[0])+fabs(size[1])<0.0075f &&
			fabs(mPreSize[0])+fabs(mPreSize[1])<0.008f &&
			fabs(mDirection)<8.0f &&
			fabs(bodyAcc.y())<2.0f &&
			fabs(bodyAcc.x())<1.5f )
		{
			//return;///////////////////////////////////////////////////////
		}

		shared_ptr<Task> mstep( new Step(isLeft,size,dir,cStep,mWalkHeight,this/*,
										 false,shouldback,mPlanTarget,isTurning,sideWalk,mIs4Kick*/) );
		mSubTaskList.push_back(mstep);
	}

	else	//local walk/////////////////////////////////////////////////////////////
	{
		const Vector2f& ballRelPos=WM.getBallRelPos2D();
		Vector2f ttSize=ballRelPos*0.5f;
		//WM.setIsLeftFootKick( ttSize.x()<0 );
		WM.setIsLeftFootKick(false);//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

		//the destination should be adjusted if it's not rushing
		if(false==WM.getRushToGoalSign())
		{
			Vector2f adjustPos(-0.026f,-0.129f);//-0.023f,-0.11f
			if(WM.getIsLeftFootKick())
				adjustPos.x()*=-1;
			ttSize+=adjustPos;
		}

		// add Step
		//printf("bodyacc:   y=%f\t x=%f\n",bodyAcc.y(),bodyAcc.x());
		/*if( fabs(ttSize[0])+fabs(ttSize[1])<0.02f
			&& fabs(mPreSize[0])+fabs(mPreSize[1])<0.02f
			&& fabs(mDirection)<8.0f
			&& fabs(bodyAcc.y())<2.0f
			&& fabs(bodyAcc.x())<1.5f )
		{
			return;
		}*/
		shared_ptr<Task> mstep( new Step(isLeft,ttSize,dir,cStep,0.35f,this/*,
										 0,0,Vector2f(0,0),0,0,0*/) );
		mSubTaskList.push_back(mstep);
	}

	LOG_FLUSH;
}


void Walk::generatePossibleState( const State& start, const State& target,
								  const State& final,
								  std::vector<State>& res,
								  std::vector<State>& unReach) const
{
	std::vector<State>::iterator iter = res.begin();
	for(; iter!=res.end(); ++iter){
		if ( iter->sameAs(target) ){
				break;
		}
	}
	if ( iter != res.end() ){
		//cout<<"target ("<<target.p()<<")is in the result"<<endl;
		return;
	}

	iter = unReach.begin();
	for(; iter!=unReach.end(); ++iter){
		if ( iter->sameAs(target) ){
				break;
		}
	}
	if ( iter != unReach.end() ){
		//cout<<"target ("<<target.p()<<")is in the unReach"<<endl;
		return;
	}

	const Vector2f& startPos = start.p();
	const Vector2f& targetPos = target.p();
	const Vector2f& finalPos = final.p();
	//cout<<"generatePossibleState ["<<startPos<<" ("<<start.costed()
	//    <<' '<<start.moreCost()<<")] ==> ["
	//    <<targetPos<<" ("<<target.costed()<<' '<<target.moreCost()<<")]"
	//    <<endl;

	Segment2f originalPath(startPos, targetPos);
	bool isBreak = false;
	FOR_EACH(iter, mBlocks){
		if ( iter->isIntersect(originalPath) ){
			//LOG_RED_LINE_2D("planPath",iter->p0(), iter->p1());
			const Vector2f& p0 = iter->p0();
			const Vector2f& p1 = iter->p1();
			State s0(p0, (p0-startPos).length()+start.costed(),
					 (p0-finalPos).length() );
			State s1(p1, (p1-startPos).length()+start.costed(),
					 (p1-finalPos).length() );

			if ( (s0.sameAs(start) && !s1.sameAs(target))
				 || (s1.sameAs(start) && !s0.sameAs(target))
				 || (s0.sameAs(target) && !s1.sameAs(start))
				 || (s1.sameAs(target) && !s0.sameAs(start))){
				//cout<<"the start or target is on the line"<<endl;
				continue;
			}

			if ( !isBreak ){
				isBreak = true;
				//cout<<"push ("<<target.p()<<" into unReach"<<endl;
				unReach.push_back(target);
			}

			//cout<<"find : ["<<p0<<", "<<p1<<"]"<<endl;


			generatePossibleState(start,s0,final,res,unReach);
			generatePossibleState(start,s1,final,res,unReach);
		}
	}

	if ( !isBreak ) {
		// there is not any intersection
		//cout<<"push the ("<<target.p()<<") in results"<<endl;
		res.push_back(target);
	}
}


void Walk::planPath()
{
	//cout<<"start path plan @ "<<WM.getSimTime()<<endl;

	mPlanTarget = mTarget;
	mPlanDir = mDirection;

	const Vector2f& myPos = WM.getMyOrigin2D();

	LOG_BLUE_LINE_2D("planPath",myPos, mTarget);

	float minCost = (myPos-mTarget).length();
	State start(myPos,0,minCost );
	State target(mTarget,minCost,0);

	setupBlocks();

	function<void (const State&, const State&, const State&, std::vector<State>&, std::vector<State>&)> genNext = bind(&Walk::generatePossibleState, this, _1, _2, _3, _4, _5);
	std::list<State> path = AStar(start, target, genNext);

	LOG_PRINTF("planPath","path size: %d",path.size());
	FOR_EACH(iter,path){
		LOG_GREEN_SPHERE("planPath",
						 Vector3f(iter->p().x(),iter->p().y(),0),0.05f);
	}

	if ( path.size()>1 ){
		std::list<State>::const_iterator iter = path.begin();
		mPlanTarget = iter->p();
		iter++;
		Vector2f nextTarget = iter->p();
		mPlanDir = (nextTarget - mPlanTarget).angle();
		mPlanTarget += (mPlanTarget - myPos).normalized();
		LOG_RED_SPHERE("planPath",
						 Vector3f(mPlanTarget.x(),mPlanTarget.y(),0),0.1f);
	}

#ifdef ENABLE_LOG
	path.push_front(start);
	FOR_EACH(iter,path){
		list<State>::const_iterator iter1 = iter;
		iter1++;
		if ( iter1 != path.end() ){
			LOG_GREEN_LINE_2D("planPath",iter->p(),iter1->p());
		}
	}
#endif // ENABLE_LOG
}

void Walk::setupBlocks()
{
	mBlocks.clear();

	// Segment2f block0(Vector2f(-1,-0.5),Vector2f(-1.0,0.5));
	// Segment2f block1(Vector2f(-1.5,0),Vector2f(-1.0,-1.5));
	// Segment2f block2(Vector2f(-0.5,-0.5),Vector2f(-1.0,1.0));
	// mBlocks.push_back(block0);
	// mBlocks.push_back(block1);
	// mBlocks.push_back(block2);

	const static float safe_dist = 0.2f;
	const Vector3f& bodySize = HUMANOID.getBoundBoxSize();
	const float self_foot_width = bodySize.x() * 0.5f + 0.1f;
	const float self_body_width = bodySize.x() * 0.5f + safe_dist;
	const float player_body_width = bodySize.x() * 0.5f;
	const float player_body_height = bodySize.z() * 0.5f;
	const float player_fall_height = player_body_height * 0.5f;

	// avoid to collide with the goals
	const static float goal_back_x = half_field_length + goal_depth;
	// left goal
	/**
	 *
	 *  \|       1    |
	 * --+------------+--
	 *   |\           |5
	 *   |
	 *   |
	 *   |
	 *   |
	 *  2|
	 *   |
	 *   |
	 *   |
	 *   |
	 *   |/           |
	 * --+------------+--
	 *  /|       3    |4
	 */
	const static Vector2f p0(-half_field_length,half_goal_width);
	const static Vector2f p1(-goal_back_x,half_goal_width);
	const static Vector2f p2(-goal_back_x,-half_goal_width);
	const static Vector2f p3(-half_field_length,-half_goal_width);

	const static float bias = self_body_width;
	const static Segment2f leftGoal1(p0+Vector2f(bias,0),
									 p1+Vector2f(-bias,0));
	const static Segment2f leftGoal2(p1+Vector2f(0,bias),
									 p2+Vector2f(0,-bias));
	const static Segment2f leftGoal3(p2+Vector2f(-bias,0),
									 p3+Vector2f(bias,0));
	// check the possibility of colliding the left goal
	Segment2f directWalk(mTarget,WM.getMyOrigin2D());
	const static Segment2f leftGoalFront(
		Vector2f(-half_field_length, goal_width),
		Vector2f(-half_field_length, -goal_width));
	const static Segment2f leftGoalBack(
		Vector2f(-half_field_length, goal_width),
		Vector2f(-half_field_length, -goal_width));
	const static Segment2f leftGoalCenter(
		Vector2f(-half_field_length, 0),
		Vector2f(-goal_back_x, 0));
	if ( leftGoalFront.isIntersect(directWalk)
		 || leftGoalBack.isIntersect(directWalk)
		 || leftGoalCenter.isIntersect(directWalk) ){
		mBlocks.push_back(leftGoal1);
		mBlocks.push_back(leftGoal2);
		mBlocks.push_back(leftGoal3);
		mBlocks.push_back(buildBlock(p0,self_body_width));
		mBlocks.push_back(buildBlock(p1,self_body_width));
		mBlocks.push_back(buildBlock(p2,self_body_width));
		mBlocks.push_back(buildBlock(p3,self_body_width));
	}

	// right goal
	const static Segment2f rightGoal1(-leftGoal1.p0(),
									  -leftGoal1.p1());
	const static Segment2f rightGoal2(-leftGoal2.p0(),
									  -leftGoal2.p1());
	const static Segment2f rightGoal3(-leftGoal3.p0(),
									  -leftGoal3.p1());
	const static Segment2f rightGoalFront(
		Vector2f(half_field_length, goal_width),
		Vector2f(half_field_length, -goal_width));
	const static Segment2f rightGoalBack(
		Vector2f(half_field_length, goal_width),
		Vector2f(half_field_length, -goal_width));
	const static Segment2f rightGoalCenter(
		Vector2f(goal_back_x, 0),
		Vector2f(half_field_length, 0));
	if ( rightGoalFront.isIntersect(directWalk)
		 || rightGoalBack.isIntersect(directWalk)
		 || rightGoalCenter.isIntersect(directWalk)){
		mBlocks.push_back(rightGoal1);
		mBlocks.push_back(rightGoal2);
		mBlocks.push_back(rightGoal3);
		mBlocks.push_back(buildBlock(rightGoal1.p0(),self_body_width));
		mBlocks.push_back(buildBlock(rightGoal1.p1(),self_body_width));
		mBlocks.push_back(buildBlock(rightGoal3.p0(),self_body_width));
		mBlocks.push_back(buildBlock(rightGoal3.p1(),self_body_width));
	}

	if (mAvoidBall){
		// avoid to collide with the ball
		//const float ball_avoid_radius = self_foot_width + ball_radius;
		float ball_avoid_radius = self_foot_width + ball_radius;
		ball_avoid_radius *= 1.2;								/////terrymimi-test
		const Vector2f& posBall = WM.getInterceptBallGlobalPos2D();//WM.getBallGlobalPos2D();
		mBlocks.push_back(buildBlock(posBall, ball_avoid_radius));
	}

	// avoid to collide with opponents
	const float fallen_player_avoid_radius = self_body_width + player_body_height;
	const float stand_player_avoid_radius = self_body_width + player_body_width;
	const map<unsigned int, Vector3f>& oppPos = WM.getOppGlobalPos();
	FOR_EACH(i, oppPos){
		const Vector3f& pos = i->second;
		if ( pos.z() < player_fall_height ){
			// avoid the fallen opponent
			mBlocks.push_back(buildBlock(Vector2f(pos.x(),pos.y()), fallen_player_avoid_radius));
			LOG_PRINTF("blocks","opponent %d fall down",i->first);
		}
	}

	// avoid to collide with teammates
	unsigned int myNum = WM.getMyUnum();
	const map<unsigned int, Vector3f>& ourPos = WM.getOurGlobalPos();
	FOR_EACH(i, ourPos){
		if ( i->first == myNum ) continue;
		const Vector3f& pos = i->second;
		if ( pos.z() < player_fall_height ){
			// avoid the teammates
			mBlocks.push_back(buildBlock(Vector2f(pos.x(),pos.y()), fallen_player_avoid_radius));
			LOG_PRINTF("blocks","teammate %d fall down",i->first);
		}
		else{
			mBlocks.push_back(buildBlock(Vector2f(pos.x(),pos.y()), stand_player_avoid_radius));
			LOG_PRINTF("blocks","teammate %d stand up",i->first);
		}
	}

	// log
	FOR_EACH(iter,mBlocks){
		LOG_RED_LINE_2D("blocks",iter->p0(),iter->p1());
	}
}

Segment2f Walk::buildBlock(const Vector2f& p, float radius) const
{
	// AngDeg ang = (p-mTarget).angle();
	AngDeg ang = mDirection;
	float avoid_x = sinDeg(ang)*radius;
	float avoid_y = cosDeg(ang)*radius;
	return Segment2f(Vector2f(p.x()+avoid_x,p.y()-avoid_y),
					 Vector2f(p.x()-avoid_x,p.y()+avoid_y));
}

Vector2f Walk::calMyOrigin2D()
{
	TransMatrixf originTrans;
	Vector2f result;
	if( 1 == mForKick )
	{
		originTrans = WM.getBoneTrans(HUMANOID.L_FOOT);
		originTrans.transfer(HUMANOID.getFootSupportBias(true));
	}
	else if( -1 == mForKick )
	{
		originTrans = WM.getBoneTrans(HUMANOID.R_FOOT);
		originTrans.transfer(HUMANOID.getFootSupportBias(false));
	}
	else
	{
		result = WM.getMyOrigin2D();
		return result;
	}

	result = Vector2f(originTrans.pos().x(), originTrans.pos().y());
	return result;
}



} // namespace task
