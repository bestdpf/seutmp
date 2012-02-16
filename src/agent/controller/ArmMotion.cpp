/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#include "ArmMotion.h"
#include "core/WorldModel.h"
#include "robot/humanoid/Humanoid.h"

namespace controller{

    using namespace boost;
    using namespace math;
    using namespace action;

    ArmMotion::ArmMotion()
    {
    }

    /**
     * this function use the FRP sensor
     * 
     */

    shared_ptr<Action> ArmMotion::control()
    {
        shared_ptr<JointAction> act(new JointAction);
        const Vector3f& Fs = WM.getFeetForce();
        //cout<<Fs<<" : ";

        float m = HUMANOID.getTotalMass();
        float mf = HUMANOID.getLegMass();
        float ma = HUMANOID.getArmMass();
        
        float fs = sqrt(pow2(Fs.x()) + pow2(Fs.y()));
        float l2 = HUMANOID.getHalfFeetWidth() * 2;
        float l1 = HUMANOID.getShoulderWidth() - l2 * 0.5f;
        const float k = 10;
        float fl = k * l2 * (2*mf) * fs / ( (2*l1 + l2) * m * ma );
        //cout<<fs<<" = "<<fl<<endl;

        unsigned int lid = HUMANOID.getJointSensorId("laj1");
        unsigned int rid = HUMANOID.getJointSensorId("raj1");

        robot::humanoid::Humanoid::ESkeleton ef = WM.getMySupportFoot();
        if ( robot::humanoid::Humanoid::L_FOOT == ef ){
            //cout<<"left leg\n";
            act->set(lid,fl);
            act->set(rid,-fl);
        }
        else if ( robot::humanoid::Humanoid::R_FOOT == ef ){
            //cout<<"right leg\n";
            act->set(lid,-fl);
            act->set(rid,fl);
        }
        
        return act;
    }

    /**
     * this function set the arm joint speed according to leg joint
     * speed directory
     * 
     */
    shared_ptr<Action> ArmMotion::control(shared_ptr<JointAction> ja)
    {
        unsigned int llid = HUMANOID.getJointSensorId("llj3");
        unsigned int rlid = HUMANOID.getJointSensorId("rlj3");
        unsigned int laid = HUMANOID.getJointSensorId("laj1");
        unsigned int raid = HUMANOID.getJointSensorId("raj1");
        unsigned int laid4 = HUMANOID.getJointSensorId("laj4");
        unsigned int raid4 = HUMANOID.getJointSensorId("raj4");
        
        AngDeg ll = 0;
        AngDeg rl = 0;
        ja->getDegree(llid, ll);
        ja->getDegree(rlid, rl);
        //cout<<"ll="<<ll<<" rl="<<rl<<endl;
        ja->set(laid,-ll);
        ja->set(raid,-rl);
        ja->set(laid4,-ll);
        ja->set(raid4,-rl);
        return ja;
    }

    /**
     * this function set the arm joint angle according to the leg
     * joint angle
     */
    void ArmMotion::control(std::map<unsigned int, math::AngDeg>& angles)
    {
        unsigned int llid = HUMANOID.getJointSensorId("llj3");
        unsigned int rlid = HUMANOID.getJointSensorId("rlj3");
        unsigned int laid = HUMANOID.getJointSensorId("laj1");
        unsigned int raid = HUMANOID.getJointSensorId("raj1");
        unsigned int laid4 = HUMANOID.getJointSensorId("laj4");
        unsigned int raid4 = HUMANOID.getJointSensorId("raj4");

        angles[laid] = -(angles[llid]-30)-80;
        angles[raid] = -(angles[rlid]-30)-80;
        angles[laid4] = -(angles[laid]+120)*0.5;
        angles[raid4] = (angles[raid]+120)*0.5;
    }

    void ArmMotion::control(std::map<unsigned int, math::AngDeg>& angles,
                            const Vector3f& pl, const Vector3f& pr, const Vector3f& pt)
    {
        Vector3f c = (pl+pr)*0.5f;

        // float lx = min(0.0f,pt.x());//pl.x() - c.x();
        // float rx = max(0.0f,pt.x());//pr.x() - c.x();
        float ly = pl.y() - c.y();
        float ry = pr.y() - c.y();
        
        unsigned int laid1 = HUMANOID.getJointSensorId("laj1");
        unsigned int raid1 = HUMANOID.getJointSensorId("raj1");
        unsigned int laid2 = HUMANOID.getJointSensorId("laj2");
        unsigned int raid2 = HUMANOID.getJointSensorId("raj2");
        unsigned int laid3 = HUMANOID.getJointSensorId("laj3");
        unsigned int raid3 = HUMANOID.getJointSensorId("raj3");
        unsigned int laid4 = HUMANOID.getJointSensorId("laj4");
        unsigned int raid4 = HUMANOID.getJointSensorId("raj4");
	/*
	//fixed angle
        angles[laid1] = -110+ry*200;//-90+ry*1200;
        angles[raid1] = -110+ly*200;//-90+ly*1200;
        angles[laid2] = 0;//rx*500;
        angles[raid2] = 0;//lx*500;
        angles[laid3] = -90;
        angles[raid3] = 90;
        angles[laid4] = -90;//-(angles[laid1]+120)*0.5;
        angles[raid4] =  90;//(angles[raid1]+120)*0.5;
	*/
	/*
	//dynamic test by dpf
	angles[laid1] = -90+ry*800;
        angles[raid1] = -90+ly*800;
        angles[laid2] = 0;//rx*500;
        angles[raid2] = 0;//lx*500;
        angles[laid3] = -90;
        angles[raid3] = 90;
        angles[laid4] = -(angles[laid1]+120)*0.5;
        angles[raid4] =  (angles[raid1]+120)*0.5;
	*/
  
	//dynamic old good
	angles[laid1] = -90+ry*1200;
        angles[raid1] = -90+ly*1200;
        angles[laid2] = 0;//rx*500;
        angles[raid2] = 0;//lx*500;
        angles[laid3] = -90;
        angles[raid3] = 90;
        angles[laid4] = -(angles[laid1]+120)*0.5;
        angles[raid4] =  (angles[raid1]+120)*0.5;

	
	
    }
} // namespace controller
