/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 * $Id: BackKick.h          Tue Jul  8 21:25:42 2008    $ 
 ****************************************************************************/
 
#ifndef TASK_BACKKICK_H
#define TASK_BACKKICK_H

#define ENABLE_TASK_BACK_KICK_LOG

#include "BasicKick.h"
#ifdef  ENABLE_TASK_BACK_KICK_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{
	using namespace std;
	
	class BackKick :public BasicKick
	{
	public:
			BackKick(const math::Vector2f& target , bool isLeftLeg = false , Task* primary = NULL);
		
	private:
			
		/********由目标点距离产生Acc时间的map*******************/
		
		virtual void generateAnkleDistTable();
				
		virtual void generateDurationDistTable();
		
		virtual AngDeg getDesiredAnkleAngle();
		
		virtual float getDesiredDuration();	
		
		DECLARE_STATIC_GRAPHIC_LOGGER; //什么意思？hu_ask
		
	};
}
#endif /* _BACKKICK_H */
