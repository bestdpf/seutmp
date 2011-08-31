/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include <sstream>
#include <fstream>
#include "Formation.h"
#include "core/WorldModel.h"
namespace configuration {

    using namespace std;
    using namespace math;

Formation::Formation()
    :mMyDataPtr(0)
{
    setupStrPlayerTypeMap();
}

Formation::~Formation()
{
}

void Formation::setupStrPlayerTypeMap()
{
	mStrPlayerTypeMap.clear();
	
	mStrPlayerTypeMap["PT_GOALKEEPER"] 		= PT_GOALKEEPER;
	mStrPlayerTypeMap["PT_DEFENDER_CENTRAL"] 	= PT_DEFENDER_CENTRAL;
	mStrPlayerTypeMap["PT_DEFENDER_SWEEPER"] 	= PT_DEFENDER_SWEEPER;
	mStrPlayerTypeMap["PT_DEFENDER_WING"] 		= PT_DEFENDER_WING;
	mStrPlayerTypeMap["PT_MIDFIELDER_CENTER"] 	= PT_MIDFIELDER_CENTER;
	mStrPlayerTypeMap["PT_MIDFIELDER_SWEEPER"] = PT_MIDFIELDER_SWEEPER;
	mStrPlayerTypeMap["PT_MIDFIELDER_WING"] 	= PT_MIDFIELDER_WING;
	mStrPlayerTypeMap["PT_ATTACKER_WING"] 		= PT_ATTACKER_WING;
	mStrPlayerTypeMap["PT_ATTACKER_CENTRAL"] 	= PT_ATTACKER_CENTRAL;
	mStrPlayerTypeMap["PT_JOYSTICK"]			= PT_JOYSTICK;
    mStrPlayerTypeMap["PT_NULL"] 			= PT_NULL;
}

void Formation::loadFile(const string& filename) throw(ClassException<Formation>)
{
    ifstream ifile(filename.c_str());
    if ( !ifile ) throw ClassException<Formation>("can not open file: "+filename);
    
    string buf;
    string content;

    while( !ifile.eof() )
    {
        getline(ifile,buf);
        content += buf.substr(0,buf.find_first_of(';'));
    }
    //cout<<"read the formation content:\n"<<content<<endl;
    
    
    pcont_t*	pcont;
    sexp_t*		sexp;
    char* c = const_cast<char*>(content.c_str());
    pcont = init_continuation(c);
    sexp = iparse_sexp(c,content.size(),pcont);

    while ( sexp )
    {
        // parse a formation
        parseOneFormation(sexp);
        
        // next formation
        destroy_sexp(sexp);
        sexp = iparse_sexp(c,content.size(),pcont);
    }
    destroy_sexp(sexp);
    destroy_continuation(pcont);

    cout<<"load formation: "<<filename<<endl;
}

void Formation::parseOneFormation(const sexp_t* sexp) throw(ClassException<Formation>)
{
    string name;
    if ( !parser::SexpParser::parseGivenValue(sexp,"formation",name) )
        throw ClassException<Formation>("can not get the formation name!");
    
    sexp = sexp->list->next->next;
    map<unsigned int, FormationData> datas;
    
    while(sexp)
    {
        unsigned int num = 0;
        if ( !parser::SexpParser::parseGivenValue(sexp,"player",num) )
            throw ClassException<Formation>(" can not get the player ");
        
        FormationData data = parseFormationData(sexp->list->next->next);
        datas[num] = data;
        sexp = sexp->next;
    }
    
    mDatas[name] = datas;
}

Formation::FormationData Formation::parseFormationData(const sexp_t* sexp) const throw(ClassException<Formation>)
{
    FormationData data;
    string typeStr;
    
    while(sexp)
    {
        if ( !parser::SexpParser::parseGivenValue(sexp,"robot",data.robot)
             && !parser::SexpParser::parseGivenValue(sexp,"type",typeStr)
             && !parser::SexpParser::parseGivenValue(sexp,"beforeKickOffBeam",
                                                     data.beforeKickOffBeam)
             && !parser::SexpParser::parseGivenValue(sexp,"ourGoalBeam",
                                                     data.ourGoalBeam)
             && !parser::SexpParser::parseGivenValue(sexp,"oppGoalBeam",
                                                     data.oppGoalBeam)
             && !parser::SexpParser::parseGivenValue(sexp,"oppGoalKick",
                                                     data.oppGoalKick)
             && !parser::SexpParser::parseGivenValue(sexp,"home",data.homePos)
                && !parser::SexpParser::parseGivenValue(sexp,"attack",data.attackPos)
                && !parser::SexpParser::parseGivenValue(sexp,"defend",data.defendPos)
             && !parser::SexpParser::parseGivenValue(sexp,"attraction",
                                                     data.attraction)
             && !parser::SexpParser::parseGivenValue(sexp,"xRange",data.xRange)
             && !parser::SexpParser::parseGivenValue(sexp,"yRange",data.yRange)
             && !parser::SexpParser::parseGivenValue(sexp,"behindBall",
                                                     data.behindBall)
            )
        {
            throw ClassException<Formation>(" a error in the formation file.");
        }
        sexp = sexp->next;
    }
    //cout<<"type "<<typeStr<<endl;
    //cout<<"beam "<<data.beamPos<<endl;
    
    data.type = getPlayerTypeByStr(typeStr);
    
    return data;
}

void Formation::setMyFormation(const string& formationName, unsigned int num) throw(ClassException<Formation>)
{
    TFormationDataMap::const_iterator iter = mDatas.find(formationName);
    if ( mDatas.end() == iter )
    {
        throw ClassException<Formation>(" can not find formation "+formationName);
    }
    map<unsigned int, FormationData>::const_iterator i = iter->second.find(num);
    if ( iter->second.end() == i )
    {
        stringstream ss;
        ss<<" can not find num "<<num<<" in the formation";
        throw ClassException<Formation>(ss.str());
    }
    mMyDataPtr = &(i->second);
}

Vector3f Formation::calStrategicPosition( const FormationData& fdata, const Vector3f& posBall )
{
	Vector3f posStrategic;
        /////////////////////////////
        Vector3f posbase;
        FormationType pft = WM.getNowFormation();

        if(pft>FT_HOME&&pft<=FT_ATTACK_LEFT){posbase = fdata.attackPos;}
        else if(pft>=FT_DEFEND_MIDDLE&&pft <=FT_DEFEND_LEFT){posbase = fdata.defendPos;}
        else{posbase = fdata.homePos;}
	posStrategic.x() = ( posbase.x() + (posBall.x()-posbase.x())*fdata.attraction.x() );
	posStrategic.y() = ( posbase.y() + (posBall.y()-posbase.y())*fdata.attraction.y() );
	//-* set max and min
	posStrategic.x() = clamp( posStrategic.x(), fdata.xRange[0], fdata.xRange[1] );
	posStrategic.y() = clamp( posStrategic.y(), fdata.yRange[0], fdata.yRange[1] );

	//-* behind ball
	if ( fdata.behindBall>0 )
	{
		posStrategic.x() = clamp( posStrategic.x(), fdata.xRange[0], posBall.x()-fdata.behindBall);
	}
	
	return posStrategic;
}

Vector3f Formation::calMyStrategicPos( const Vector3f& posBall ) const
{
    if ( 0!= mMyDataPtr ){
        return calStrategicPosition(*mMyDataPtr, posBall);
    }
    cout << "No position"<<endl;
    return Vector3f(0,0,0);
}


} // namespace configuration
