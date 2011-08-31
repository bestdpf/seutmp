/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "humanoid/Nao.h"
#include "humanoid/SoccerBot.h"
#include "importer/RSGImporter.h"
#include <sys/time.h>

using namespace std;
using namespace boost;
using namespace tree;
using namespace robot;
using namespace robot::device;
using namespace robot::humanoid;
using namespace robot::importer;


int main(int argc, char* argv[])
{
    cout<<"************* Robot Lib Test ****************"<<endl;
    try{
        cout<<"\n======== Soccerbot 056 ===========\n"<<endl;
        ROBOT.init<SoccerBot056>();
        cout<<"\n======== Soccerbot 058 ===========\n"<<endl;
        ROBOT.init<SoccerBot058>();
        cout<<"\n======== Nao =========\n"<<endl;
        ROBOT.init<Nao>();
    }
    catch(ClassException<RSGImporter>& e){
        cerr<<e.what()<<endl;
    }
    catch(ClassException<Node<Device> >& e){
        cerr<<e.what()<<endl;
    }

    cout<<"************** Test Inverse Kinematics ******************\n";
    // cout.setf(ios_base::fixed,ios_base::floatfield);
    // cout.precision(2);
    int count = 0;
    timeval tim;
    gettimeofday(&tim, NULL);
    double starttime=tim.tv_sec+(tim.tv_usec/1000000.0);
    while ( count < /*10*60*50*/1 ){
        count++;
        TransMatrixf dFootR, dFootL, dBody;
        dFootR.identity();
        dFootR.p().x() = HUMANOID.getHalfFeetWidth();
        dFootR.p().z() = HUMANOID.getMinFootHeight();
        dFootL.identity();
        dFootL.p().x() = -HUMANOID.getHalfFeetWidth();
        dFootL.p().z() = HUMANOID.getMinFootHeight();
        dBody.rotationX(-10);
        dBody.pos() = Vector3f(0,0,0.35f);
    
        std::map<unsigned int, math::AngDeg> originalAngles;
        originalAngles.clear();
        for( unsigned int i=0; i<26; i++){
            originalAngles[i] = random(-90.0f,90.0f);
        }

        std::map<unsigned int, math::AngDeg> angles;
        angles.clear();
        for( unsigned int i=0; i<26; i++){
            angles[i] = random(-90.0f,90.0f);//originalAngles[i] + random(-7.0f,7.0f);
        }
        
        list<shared_ptr<const Bone> > idxL, idxR;
        HUMANOID.findRoute(Humanoid::TORSO, Humanoid::L_FOOT, idxL);
        HUMANOID.findRoute(Humanoid::TORSO, Humanoid::R_FOOT, idxR);
        FOR_EACH( iter, idxL ){
            (*iter)->restricJointAngles(originalAngles);
            (*iter)->restricJointAngles(angles);
        }
        FOR_EACH( iter, idxR ){
            (*iter)->restricJointAngles(originalAngles);
            (*iter)->restricJointAngles(angles);
        }

        if ( HUMANOID.legInverseKinematics(true, dBody, dFootL, angles)
             && HUMANOID.legInverseKinematics(false, dBody, dFootR, angles)){
            FOR_EACH( iter, angles ){
                cout<<HUMANOID.getJointSensorName(iter->first)
                    <<' '<<iter->second<<' ';
            }
            cout<<endl;
        }
        else{
            cerr<<"error"<<endl;
        }
    
        
    }
    gettimeofday(&tim, NULL);
    double endtime=tim.tv_sec+(tim.tv_usec/1000000.0);
    printf("%.6lf seconds elapsed\n", endtime-starttime);
    return 0;
}
