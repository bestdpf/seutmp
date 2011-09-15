/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "Perception.h"
#include "Template.hpp"
#include "../action/JointAction.h"
#include "../action/Actions.h"

#include <fstream>

namespace perception {

using namespace std;
using namespace boost;
using namespace action;


Perception::Perception()
{
}


Perception::Perception(const std::string& msg)
{
	//printf("%s\n",msg.c_str());
    pcont_t*	pcont;
	sexp_t*		sexp;
	char* c = const_cast<char*>(msg.c_str());
	pcont = init_continuation(c);
 	sexp = iparse_sexp(c,msg.size(),pcont);

    do
    {
        const sexp_t* t = sexp->list;
        if ( parser::SexpParser::isVal(t) )
        {
			bool ok = true;
			string name(t->val);
            if ( "HJ" == name ) ok = mJoints.updateHJ(t->next);// hinge joint
            else if ( "UJ" == name ) ok = mJoints.updateUJ(t->next);// universal joint
			else if ( "TCH" == name ){// touch
                mTouch = boost::shared_ptr<Touch>(new Touch);
                ok = mTouch->update(t->next);
            }
			else if ( "FRP" == name ) ok = mForceResistance.update(t->next); // force resistance
            else if ( "See" == name ){// See
				//==================================TT test
                mVision = boost::shared_ptr<Vision>(new Vision);
                ok = mVision->update(t->next);
				//ok=true;
				//=================================================
            }
            else if ( "time" == name ) ok = mTime.update(t->next); // time
			else if ( "GYR" == name ) ok = mGyroRate.update(t->next); // gyro rate
			else if ( "ACC" == name ) ok = mAccelerometer.update(t->next); //  Accelerometer add by allen 2010-03-15
			else if ( "GS" == name ) ok = mGameState.update(t->next); // game state
            else if ( "hear" == name ){ // hear
                boost::shared_ptr<Hear> hear(new Hear);
                if ( hear->update(t->next) )
                    mHear.push_back(hear);
            }

			else std::cerr<<" Perception unknow name: "<<string(t->val)<<'\n';
			if ( !ok ) std::cerr<<" Perception update failed: "<<string(t->val)<<'\n';
        }
        destroy_sexp(sexp);
        sexp = iparse_sexp(c,msg.size(),pcont);
    } while ( sexp );

    destroy_sexp(sexp);
	destroy_continuation(pcont);
}


std::ostream& operator<<(std::ostream &stream, const Perception& p)
{
	stream<<p.mTime<<'\n';
    if ( NULL != p.mVision.get() )
        stream<<*p.mVision<<'\n';
	stream<<p.mJoints<<'\n';
	stream<<p.mGyroRate<<'\n';
        ////////////////////////////////////////////add by allen 2010.3.15
        stream<<p.mAccelerometer<<'\n';
    if ( NULL != p.mTouch.get() )
        stream<<*p.mTouch<<'\n';
    stream<<p.mForceResistance<<'\n';
    return stream;
}

bool Perception::predict(shared_ptr<const Action> act, float t, bool advanceTime)
{
	if( advanceTime ){
		// set the time to the next cycle
		mTime.now() += t;
		mTime.step() = t;
	}

	// multi-actions
	shared_ptr<const Actions> acts= shared_dynamic_cast<const Actions>(act);
	if( NULL != acts.get() )
	{
		const Actions::TActionPtrs& actions = acts->get();
		FOR_EACH(iter,actions)
		{
			if( !predict(*iter, t, false) ){
				return false;
			}
		}
	}

	// assume the last action was executed
	shared_ptr<const JointAction> jact = shared_dynamic_cast<const JointAction>(act);
	if( 0!=jact.get() )
	{
		if(jact->isBodyJointAction())
			mJoints.predict(jact, t);
	}

	return true;
}


bool Perception::update( const Perception& pp )
{
	float dt = time().now() - pp.time().now();
	if ( dt < serversetting::sim_step/2 ) return false;

	mJoints.updateRate( pp.joints(), dt );

	return true;
}


} //end of namespace perception

