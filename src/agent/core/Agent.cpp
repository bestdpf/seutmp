/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include <boost/bind.hpp>
#include "CommUnit.hpp"
#include "WorldModel.h"
#include "configuration/Configuration.h"
#include "action/JointAction.h"
#include "Agent.h"


namespace core {

using namespace boost;
using namespace action;
using namespace perception;

Agent::Agent()
	:mIsAlive(false)
{
}

Agent::~Agent()
{
}

bool Agent::init()
{
	/// we set the offclient file by the `standard' path and name
	stringstream ssstream;
	ssstream << "logs/"
			 << OPTS.arg<string>("teamname")
			 << "-" << OPTS.arg<unsigned int>("unum")
			 << "/"
			 <<OPTS.arg<string>("server")<<".log";
	const string& file = ssstream.str();

	if ( OPTS.arg<int>("port") > 0 ){ // on-line mode
		try{
			COMM.connect(OPTS.arg<string>("server"), OPTS.arg<int>("port"));
		}
		catch(ClassException<net::Socket>& e){
			cerr<<e.what()<<endl;
			return false;
		}
		/// see if the messages should be logged
		if ( OPTS.count("offclient") ){
			mSenseLog.open(file.c_str());
			if ( !mSenseLog ){
				cerr<<"can not open file "<<file
					<<" for offclient logging"<<endl;
			}
			else{
				cout<<"write messages to file "<<file<<endl;
			}
		}
	}
	else{ //off-line mode
		try{
			COMM.connect(file, -1);
		}
		catch(ClassException<CommUnit>& e){
			cerr<<e.what()<<endl;
			return false;
		}
		cout<<"Start as offcilent, from file: "<<file<<endl;
	}

	// send the create message
	cout<<"Create Robot: "<<FM.getMy().robot<<endl;
	COMM<<"(syn)(scene "<<FM.getMy().robot<<")";
	COMM.send();

	xtime_get(&mLastSenseTime, TIME_UTC);
	xtime_get(&mLastActTime, TIME_UTC);
	mIsAlive = true;
	return true;
}


void Agent::run()
{
	while ( true )
	{
		// get sense from the server
		shared_ptr<Perception> p = sense();
		if ( 0 == p.get() )
			break;

		// update the world model
		if(!WM.update(p))
			break;
		// make the decision
		shared_ptr<Action> act = think();

		// perform the actions
		perform(act);
	}
	cerr<<"[ERROR] Agent::run receive nothing!"<<endl;
}


void Agent::done()
{
	cout <<"Simulation Finished!"<<endl;
	// do nothing
}


boost::shared_ptr<Perception> Agent::sense()
{
	std::string msg;
	/// get the message from the server
	try{
		COMM>>msg;
	}
	catch(ClassException<net::Socket>& e){
			cerr<<e.what()<<endl;
	}

	if ( msg.empty() )
		return shared_ptr<Perception>();

	if ( mSenseLog ){
		mSenseLog<<msg<<endl;
	}
//	cout<<msg<<endl;                                                   //just for test

	shared_ptr<Perception> p( new Perception(msg) );
	return p;
}


void Agent::perform( shared_ptr<Action> a )
{
	if(0!=a.get())
	{
		xtime_get(&mLastActTime, TIME_UTC);
		COMM<<a->command();
		//cout<<a->command()<<endl;//////////////////////////////////////TT test
		COMM<<"(syn)";
		COMM.send();
		mutex::scoped_lock lock(mLastActMutex);
		mLastAction = a;
	}
	else
	{
		cerr<<"[Error] do not know what to do!"<<endl;
	}
}


void Agent::runMultiThreads()
{
    //single thread test
    
	cout<<"Start run threads"<<endl;

	thread_group ctrThrdGroup;
	/// create the sense thread
	ctrThrdGroup.create_thread(bind(&Agent::senseThread, this));
	/// create the act thread
	ctrThrdGroup.create_thread(bind(&Agent::actThread, this));

	thinkThread();

	// wait for threads
	ctrThrdGroup.join_all();
}


void Agent::thinkThread()
{
	while(mIsAlive)
	{
		shared_ptr<Perception> p;
		{
			mutex::scoped_lock lock(mPerMutex);

			if ( 0 == mPer.get() ){
				/// there is no perception currently
				mPerCond.wait(lock);
			}
			p = mPer;
			mPer.reset();
		}

		if ( 0 != p.get() )		// there are new message
		{
			if ( WM.update(p) ) // update the world model
			{
				// make the decision
				shared_ptr<Action> a = think();
				mutex::scoped_lock lock(mActMutex);
				mAct = a;
			}
			else	//update failed
			{
				cerr<<"[Think Thread] the perception @ "<<p->time().now()<<" is skipped."<<endl;
			}
		}
		else					// no new message come in time
		{
			if ( WM.update(p) ){
				cerr<<"[Think Thread] think without sense @ "
					<<WM.getSimTime()<<endl;
				// make the decision
				//mAct = think();
				shared_ptr<JointAction> jact ( new JointAction );	//TT: it has been initialized
				//jact->fill(0);
				mutex::scoped_lock lock(mActMutex);
				mAct = shared_static_cast<Action>(jact);
			}
			else{
				// failed to update from old perception and last action
				cerr<<"[Think Thread] the generatored perception @ "<<p->time().now()<<" is failed."<<endl;
			}
		}
	}
}


void Agent::senseThread()
{
	while( mIsAlive )
	{
		// get sense from the server
		shared_ptr<Perception> p = sense();

		if ( 0 == p.get() ){
			break;
		}

		/// lock the mutex and then change the perception cache
		mutex::scoped_lock lock(mPerMutex);
		if ( 0 != mPer.get() )
			cerr<<"[Warning] the perception @ "<<mPer->time().now()<<" is dropped!"<<endl;
		mPer = p;
		{
			mutex::scoped_lock timeLock(mTimeMutex);
			xtime_get(&mLastSenseTime, TIME_UTC);
			mTimeCond.notify_one();
		}
		/// notify the think thread that the perception has been updated
		mPerCond.notify_one();
	}

	mIsAlive = false;
	mPerCond.notify_one();
}


void Agent::actThread()
{
	while( mIsAlive )
	{
		{
			mutex::scoped_lock lock(mTimeMutex);
			mTimeCond.wait(lock);
			calculateNextActTime();
		}
		// wait until act time reach
		boost::thread::sleep(mNextActTime);

		mutex::scoped_lock lock(mActMutex);
		perform(mAct);
	}
}


/**
 * a help function to change xtime
 *
 * @param t the xtime to be changed
 * @param nsec the detla time in nanosecond
 */
void changeXTime(xtime& xt, int nsec)
{
	xt.nsec += nsec;
	const int second = 1000000000;
	if ( xt.nsec > second ){
		xt.nsec -= second;
		xt.sec++;
	}
}


void Agent::calculateNextActTime()
{
	//cerr<<"LST= "<<mLastSenseTime.sec<<' '<<mLastSenseTime.nsec<<'\t';
	//cerr<<"LAT= "<<mLastActTime.sec<<' '<<mLastActTime.nsec<<'\t';
	mNextActTime = mLastSenseTime;
	changeXTime(mNextActTime, 10000000);
	while ( mLastActTime.sec > mNextActTime.sec ||
			(mLastActTime.sec == mNextActTime.sec
			 && mLastActTime.nsec > mNextActTime.nsec) ){
		changeXTime( mNextActTime, 20000000 );
	}
	//cerr<<"NAT= "<<mNextActTime.sec<<' '<<mNextActTime.nsec
	//    <<" @ "<<WM.getSimTime()<<endl;
}


} // end of namespace core

