/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "JointPerception.h"
#include "robot/humanoid/Humanoid.h"
#include<stdio.h>///////////////////////////////////////////

namespace perception {

using namespace std;
using namespace boost;
using namespace math;
using namespace action;


bool JointPerception::update( const std::string& msg )
{
	pcont_t*	pcont;
	sexp_t*		sexp;
	char* c = const_cast<char*>(msg.c_str());
	pcont = init_continuation(c);
 	sexp = iparse_sexp(c,msg.size(),pcont);

    do
    {
        update(sexp);

        destroy_sexp(sexp);
        sexp = iparse_sexp(c,msg.size(),pcont);
    } while ( sexp );

    destroy_sexp(sexp);
	destroy_continuation(pcont);
	return true;
}

bool JointPerception::update( const sexp_t* sexp )
{
	const sexp_t* t = sexp->list;
	if ( parser::SexpParser::isVal(t) )
	{
		switch ( *(t->val) )
		{
			case 'H': updateHJ(t->next);
			break;
			case 'U': updateUJ(t->next);
			break;
			default: return false;
			break;
		}
	}
	return true;
}

bool JointPerception::updateHJ(const sexp_t* sexp)
{
    // get the name
    std::string name;
    if ( !parser::SexpParser::parseGivenValue(sexp,SS_NAME_STR,name) )
	{
		std::cerr<<"can not get the HJ name!\n";
		return false;
	}

	// get the id
    int jid = HUMANOID.getJointSensorId(name);
    if ( jid < 0 )
	{
		std::cerr<<"unknown the HJ name!"<<name<<'\n';
		return false;
	}

	// set the joint
    sexp = sexp->next;
    mJointMap[jid] = Joint(sexp, sexp->next);
	return true;
}

bool JointPerception::updateUJ(const sexp_t* sexp)
{
    // get the name
    std::string name;
    if ( !parser::SexpParser::parseGivenValue(sexp,SS_NAME_STR,name) )
	{
		std::cerr<<"can not get the UJ name!\n";
		return false;
	}

	// get the id_1 and id_2
    int jid1 = HUMANOID.getJointSensorId(name);
    int jid2 = jid1 + 1;
	 if ( jid1 < 0 )
	{
		std::cerr<<"unknown the UJ name!"<<name<<'\n';
		return false;
	}

	// set the joint1 and joint2
	sexp = sexp->next;
    const sexp_t* sexp2 = sexp->next;
    if ( 0 != sexp2->next ){ // ax & rt
        mJointMap[jid1] = Joint(sexp, sexp->next);
        sexp2 = sexp2->next;
        mJointMap[jid2] = Joint(sexp2, sexp2->next);
    }
    else{ // only ax
        mJointMap[jid1] = Joint(sexp);
        mJointMap[jid2] = Joint(sexp2);
    }



	return true;
}

AngDeg JointPerception::jointAng( unsigned int jid ) const
{
	TJointMap::const_iterator iter = mJointMap.find(jid);
	if ( iter != mJointMap.end() ) return iter->second.angle();
    // cerr<<"[Joint Perception] error: can not find joint angle"<<jid<<endl;
	return 0;
}

AngDeg JointPerception::jointRate( unsigned int jid ) const
{
	TJointMap::const_iterator iter = mJointMap.find(jid);
	if ( iter != mJointMap.end() ) return iter->second.rate();
    // cerr<<"[Joint Perception] error: can not find joint rate"<<jid<<endl;
	return 0;
}

void JointPerception::updateRate( const JointPerception& last, float deltaTime )
{
	if( deltaTime > 0 )
	{
		float invDeltaTime = 1.0f / deltaTime;
		for( TJointMap::iterator iter=mJointMap.begin();
             iter!=mJointMap.end(); ++iter )
    	{
			float deltaAng = calClipAng( iter->second.angle(),
                                         last.jointAng(iter->first) );
        	iter->second.setRate( deltaAng * invDeltaTime );
    	}
	}
	else // deltaTime <=0, so let the rates qeual to the last rates
	{
		for( TJointMap::iterator iter=mJointMap.begin();
             iter!=mJointMap.end(); ++iter )
    	{
        	iter->second.setRate(last.jointRate(iter->first));
    	}
	}
}

void JointPerception::exchange()
{
	// we do this handly, may be we should make it auto
    swap(mJointMap[2],mJointMap[6]);
    swap(mJointMap[3],mJointMap[7]);
    mJointMap[3].angReverse();
    mJointMap[7].angReverse();
    swap(mJointMap[4],mJointMap[8]);
    mJointMap[4].angReverse();
    mJointMap[8].angReverse();
    swap(mJointMap[5],mJointMap[9]);
    mJointMap[5].angReverse();
    mJointMap[9].angReverse();
    swap(mJointMap[10],mJointMap[16]);
    swap(mJointMap[11],mJointMap[17]);
    mJointMap[11].angReverse();
    mJointMap[17].angReverse();
    swap(mJointMap[12],mJointMap[18]);
    swap(mJointMap[13],mJointMap[19]);
    swap(mJointMap[14],mJointMap[20]);
    swap(mJointMap[15],mJointMap[21]);
    mJointMap[15].angReverse();
    mJointMap[21].angReverse();
    /*
	swap(mJointMap[JID_LEG_L_1],mJointMap[JID_LEG_R_1]);
	mJointMap[JID_LEG_L_1].angReverse();
	mJointMap[JID_LEG_R_1].angReverse();
	swap(mJointMap[JID_LEG_L_2],mJointMap[JID_LEG_R_2]);
	swap(mJointMap[JID_LEG_L_3],mJointMap[JID_LEG_R_3]);
	mJointMap[JID_LEG_L_3].angReverse();
	mJointMap[JID_LEG_R_3].angReverse();
	swap(mJointMap[JID_LEG_L_4],mJointMap[JID_LEG_R_4]);
	swap(mJointMap[JID_LEG_L_5],mJointMap[JID_LEG_R_5]);
	swap(mJointMap[JID_LEG_L_6],mJointMap[JID_LEG_R_6]);
	mJointMap[JID_LEG_L_6].angReverse();
	mJointMap[JID_LEG_R_6].angReverse();
	swap(mJointMap[JID_ARM_L_1],mJointMap[JID_ARM_R_1]);
	mJointMap[JID_ARM_L_1].angReverse();
	mJointMap[JID_ARM_R_1].angReverse();
	swap(mJointMap[JID_ARM_L_2],mJointMap[JID_ARM_R_2]);
	mJointMap[JID_ARM_L_2].angReverse();
	mJointMap[JID_ARM_R_2].angReverse();
	swap(mJointMap[JID_ARM_L_3],mJointMap[JID_ARM_R_3]);
	mJointMap[JID_ARM_L_3].angReverse();
	mJointMap[JID_ARM_R_3].angReverse();
	swap(mJointMap[JID_ARM_L_4],mJointMap[JID_ARM_R_4]);*/
}

void JointPerception::zero()
{
    for( unsigned int i=0; i<HUMANOID.degreeOfFreedom(); i++)
    {
        mJointMap[i].setAngle( 0 );
    }
}


std::ostream& operator<<(std::ostream &stream, const JointPerception& p)
{
    /*
	stream<<"[J ";
    for( JointPerception::TJointMap::const_iterator iter=p.mJointMap.begin(); iter!=p.mJointMap.end(); 	++iter )
    {
		stream<<"["SS_NAME_STR" " <<SS.getPerceptorName(iter->first)<<"]"<<iter->second<<"]";
    }
	stream<<']';
	*/
    for( JointPerception::TJointMap::const_iterator iter=p.mJointMap.begin(); iter!=p.mJointMap.end(); 	++iter )
    {
		stream<<HUMANOID.getJointSensorName(iter->first)<<' '<<iter->second.angle()<<'\t';
    }
	return stream;
}

std::istream& operator>>(std::istream &stream, JointPerception& p)
{
	while( !stream.eof() )
	{
		std::string name;
		float val=0;
		stream>> name >> val;
		if ( name.empty() ) break;
		p.mJointMap[HUMANOID.getJointSensorId(name)].setAngle(val);
	}

	return stream;
}


bool JointPerception::predict( shared_ptr<const JointAction> jact, float t )
{
	map<unsigned int, AngDeg> angles; //it doesn't care about head joints here
	AngDeg rt;
	unsigned int jid;

	FOR_EACH(iter, mJointMap)
	{
		jid=iter->first;
		if(jid<=1){
			angles[jid]=0;
		}
		else if( jact->getDegree(jid, rt) ){
			angles[jid] = iter->second.angle() + ( rt * t );
		}
		else{
			angles[jid] = iter->second.angle() ;
			//printf("JP.cpp L257 false\n");
		}
	}

	HUMANOID.restricJointAngles(angles);

	float ti=1/t;
	FOR_EACH(iter, mJointMap)
	{
		AngDeg ax = angles[iter->first];
		AngDeg rt = calClipAng(ax, iter->second.angle())*ti;
		iter->second.setAngle(ax);
		iter->second.setRate(rt);
	}

	return true;
}


map<unsigned int, AngDeg> JointPerception::jointAngles() const
{
	map<unsigned int, AngDeg> angles;
	FOR_EACH( iter, mJointMap ){
		angles[iter->first] = iter->second.angle();
	}
	return angles;
}


} //end of namespace perception

