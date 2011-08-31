/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/ 

#ifndef TASK_PASSSIDELONG_H
#define TASK_PASSSIDELONG_H

#define ENABLE_TASK_PASSSIDELONG_LOG

#include "BasicKick.h"

#ifdef ENABLE_TASK_PASSSIDELONG_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{
	
	class PassSidelong: public BasicKick
	{
	public:
			PassSidelong( const math::Vector2f& target, bool isLeftLeg = false,Task* primary = NULL );
				
	private:
			
		/*
		* 生成踢球脚ankle角度到球运动距离的map
		*/
		virtual void generateAnkleDistTable();
				
		virtual void generateDurationDistTable();
			
		virtual AngDeg getDesiredAnkleAngle();	
				
		virtual float getDesiredDuration();	
			
        DECLARE_STATIC_GRAPHIC_LOGGER;
			
	};
	
}

#endif /* PASSSIDELONG_H */

