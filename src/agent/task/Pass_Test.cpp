/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: 
 *
 ****************************************************************************/
 
#include "Pass_Test.h"
#include "core/WorldModel.h"
#include "SoccerDefines.h"
#include <fstream>
 
 namespace task{
	 using namespace serversetting;
	 using namespace math;
	 
	 PassTest::PassTest(Task* primary):Task(-1, primary)
	 {
         
	 }
	 
	 /** 
      * 
      * @param samplingDistance distance between two neighboring tested points on the ball's surface
      * 
     */
	 void PassTest::generateTestContactPointSet(float samplingDistance)
	 {
		 const float CIRCUMFERENCE = 2.0 * M_PI * serversetting::ball_radius;
		 Vector3f posBall3D = WM.getBallGlobalPos();
		 Vector2f posBall2D = WM.getBallGlobalPos2D();
		 Vector2f posBallRelativeToTargetPoint = posBall2D - mTargetPos;
		 AngRad verticalDeltaAng = samplingDistance * 2.0 * M_PI / CIRCUMFERENCE;
		 
		 AngRad horizontalAng = deg2Rad(atanDeg(posBallRelativeToTargetPoint.getX() 
		 						/ posBallRelativeToTargetPoint.getY()));
		 AngRad verticalAng = (0.0 - M_PI / 2.0);
		 
		 while(verticalAng <= 0.0)
		 {
			 Vector3f polar(ball_radius, horizontalAng, verticalAng);
			 Vector3f contactPoint = pol2xyz(polar);
			 mTestContactPointSet.push_back(contactPoint);
			 verticalAng += verticalDeltaAng;
		 }
		 
		 writeTestPointsToFile();
	 }
	 
	 /**
	  *
	  * @brief write position tested points to file "tested points.dat" in the form ［x,y,z］
 	  *
	 */
	 void PassTest::writeTestPointsToFile()
	 {
		 ofstream testPointDataFile("tested points.dat");
		 Vector3f contactPoint;
		 vector<Vector3f>::iterator iter;
		 
		 for(iter = mTestContactPointSet.begin(); iter != mTestContactPointSet.end(); iter ++)
		 {
			 contactPoint = *iter;
			 testPointDataFile << "[" << contactPoint.x() << "," 
			 			<< contactPoint.y() << ","
			 			<< contactPoint.z() << "] ";
		 }
		 
		 testPointDataFile.close();
	 }
 
 }	//namespace task
