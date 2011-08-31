/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id: PassAng45.h Thu Jul  3 22:03:00 2008   $
 *
 ****************************************************************************/


#ifndef TASK_PASSANG_H
#define TASK_PASSANG_H

#define ENABLE_TASK_PASSANG45_LOG

#include "BasicKick.h"
#ifdef ENABLE_TASK_PASSANG45_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif


namespace  task {
	
	using namespace std;
	
	class PassAng45 : public BasicKick
	{
     public:
		PassAng45( const math::Vector2f& target, bool isLeftLeg = false,Task* primary = NULL );
	 
	 private:
		
		/*
		* 生成踢球脚ankle角度到球运动距离的map
		*/
		virtual void generateAnkleDistTable();
				
		virtual void generateDurationDistTable();
			
		virtual AngDeg getDesiredAnkleAngle();	
				
		virtual float getDesiredDuration();	
	 
	 private:
				
		static const float mBiasAngle;
        
        DECLARE_STATIC_GRAPHIC_LOGGER;
	 };
	
}
#endif
