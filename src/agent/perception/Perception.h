/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/

#ifndef PERCEPTION_PERCEPTION_H
#define PERCEPTION_PERCEPTION_H

#include "Time.h"
#include "Vision.h"
#include "JointPerception.h"
#include "GyroRate.h"
#include "Accelerometer.h"
#include "Touch.h"
#include "GameState.h"
#include "ForceResistance.h"
#include "Hear.h"
#include "../action/Action.h"

namespace perception {


class Perception
{
public:

    Perception();

    Perception(const std::string& msg);

	~Perception(){};

	//  query functions, read only
	const Time& time() const { return mTime; }

    boost::shared_ptr<const Vision> vision() const { return mVision; }

	const JointPerception& joints() const { return mJoints; }

	const GameState& gameState() const { return mGameSate; }

    boost::shared_ptr<const Touch> touch() const { return mTouch; }

	const ForceResistance& forceResistance() const { return mForceResistance; }

    std::vector<boost::shared_ptr<Hear> > hear() const { return mHear; }

	const GyroRate& gyroRate() const
	{
		return mGyroRate;
	}

	const Accelerometer& accelerometer() const
	{
		return mAccelerometer;
	}

	void setPlayMode(serversetting::TPlayMode pm)
	{
		mGameSate.setPlayMode(pm);
	}

	serversetting::TPlayMode getPlayMode() const
	{
		return mGameSate.getPlayMode();
	}

    /**
     * stream function
     *
     * @param stream the output stream
     * @param p a perception which be covert
     *
     * @return the result stream
     */
	friend std::ostream& operator<<(std::ostream &stream, const Perception& p);


    /**
     * predict the new perception from the last action
     *
     * @param act the last action
     * @param t the time be predicted
     * @param advanceTime whether advance the time
     *
     * @return if the predict successfully
     */
    bool predict(boost::shared_ptr<const action::Action> act,
                 float t, bool advanceTime=true);

    /**
     * update the perception information according previous perception
     *
     * @param pp the previous perception
     *
     * @return if the update successfully
     */
    bool update( const Perception& pp );


private:

	/** perception of vision */
    boost::shared_ptr<Vision> mVision;

    /** perception of hear */
    std::vector<boost::shared_ptr<Hear> > mHear;

	/** perception of time */
	Time mTime;

	/** perception of joints */
	JointPerception mJoints;

	/** perception of gyrorate */
	GyroRate mGyroRate;
        ////////////////////add by allen 2010.3.15
        /** perception of Accelerometer */
	Accelerometer mAccelerometer;
/////////////////////////////////
	/** perception of touch */
    boost::shared_ptr<Touch> mTouch;

	/** game state */
	GameState mGameSate;

	/** perception of force resistance */
	ForceResistance mForceResistance;
};

std::ostream& operator<<(std::ostream &stream, const Perception& p);

} // namespace perception

#endif // PERCEPTION_PERCEPTION_H
