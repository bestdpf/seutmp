
#include "Joystick.h"
#include <iostream>

#ifdef BUILD_JOYSTICK

namespace controller {

using namespace std;

Joystick::Joystick(int id):mID(id)
{
	//Initialisation of Joystick
	if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK) < 0)
	{
		std::cerr<<"Couldn't initialize SDL: "<<SDL_GetError()<<"\n";
		exit(1);
	}
	SDL_JoystickEventState(SDL_ENABLE);
	cout<<"Found "<<SDL_NumJoysticks()<<" joysticks in this computer.\n";
	mJoystick = SDL_JoystickOpen(mID);
	cout<<"Open Joystick: "<<SDL_JoystickName(mID);
	cout<<" which has "<<SDL_JoystickNumAxes(mJoystick)<<" axis, "<<SDL_JoystickNumButtons(mJoystick)<<" buttons, "
	   <<SDL_JoystickNumBalls(mJoystick)<<" balls and "<<SDL_JoystickNumHats(mJoystick)<<" hats.\n";
	   
	mDemo = new controller::DemoStop;
}

Joystick::~Joystick()
{
	//Closing Joystick subsystem
	SDL_JoystickClose(mJoystick);
	SDL_QuitSubSystem(SDL_INIT_VIDEO|SDL_INIT_JOYSTICK);
}

boost::shared_ptr<action::Action> Joystick::play(const perception::Perception& p)
{
	// initialize moving Vectors
	int move_x;
	int move_y;
	//int move_lr;
	//int move_ud;
	
	// event loop
	SDL_Event event;
	if(SDL_PollEvent(&event))
	{
		switch (event.type)
		{
		    case SDL_QUIT: exit(1);
		    break;
			case SDL_JOYAXISMOTION: //(all Events tested on BTP-C036)
				switch(event.jaxis.axis){
					// slow but precise move left-right
					case 0:
					move_x= SDL_JoystickGetAxis(mJoystick,0);
					std::cout<<"axis 0\n";
					delete mDemo;
					mDemo = new controller::DemoWalk;//DemoTurnToBall;
	                break;
	 
		            // slow but precise move up-down
		            case 1:
	                move_y= SDL_JoystickGetAxis(mJoystick,1);
	                //move_ud= -(move_y/500);
	                //drive(Vector3f(0,move_ud,0));
					std::cout<<"axis 1\n";
	                break;
                    // fast but unprecise move up-down
                    case 2:
					move_y= SDL_JoystickGetAxis(mJoystick,2);
					//move_ud= -(move_y/100);
					//drive(Vector3f(0,move_ud,0));
					std::cout<<"axis 2\n";
					break;
					// fast but unprecise move left-right
	                case 3:
					move_x= SDL_JoystickGetAxis(mJoystick,3);
					//move_lr= move_x/100;
					//drive(Vector3f(move_lr,0,0));
					std::cout<<"axis 3\n";
					break;
 
                    default:
						//drive(Vector3f(0,0,0));
                          std::cout<< "axis motion default\n";
                          break;
                          }
             break;

			// events which occur if certain buttons are pressed
			case SDL_JOYBUTTONDOWN:
			     delete mDemo;
			     cout<<"JOYBUTTONDOWN "<<static_cast<int>(event.jbutton.button)<<'\t';
				switch(event.jbutton.button)
				{
				    case 1:
				    cout<< "squat\n";
				    mDemo = new controller::DemoSquat;
				    break;
				    
				    case 2:
				    cout<<"rise\n";
				    mDemo = new controller::DemoRise;
				    break;
					
					case 3:
					cout<<"jump\n";
					mDemo = new controller::DemoJump;
				    break;
				    
				    case 4:
				    cout<<"car\n";
				    mDemo = new controller::DemoCar;
				    break;
					
					case 5:
				    cout<<"standup\n";
				    mDemo = new controller::DemoStandup;
				    break;
					
					case 6:
				    cout<<"single leg\n";
				    //mDemo = new controller::DemoSingleLeg;
				    break;
					
					case 9:
					cout<< "stop\n";
				    mDemo = new controller::DemoStop;
					break;
					
					default:
					mDemo = new controller::DemoStop;
					break;
				}
					
			break;
					
			default:
			break;
		}
	}
	return mDemo->control(p);
}

} // namespace controller

#endif // BUILD_JOYSTICK
