


#ifndef CONTROLLER_JOYSTICK_H
#define CONTROLLER_JOYSTICK_H

//#include <config.h>

#ifdef BUILD_JOYSTICK

#include <SDL/SDL.h>
#include "Demo.h"

namespace controller
{

class Joystick
{
public:

    Joystick(int id=0);
    
    ~Joystick();
    
    bool init();
    
	void destory();
	
	boost::shared_ptr<action::Action> play(const perception::Perception& p);
	
private:
	SDL_Joystick *mJoystick;
	
	int mID;
	
	bool isJoystickLive;
	
	controller::Demo *mDemo;
};

} // namespace controller

#endif // BUILD_JOYSTICK

#endif // CONTROLLER_JOYSTICK_H
