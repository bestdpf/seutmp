
#ifndef PERCEPTION_JOINT_PERCEPTION_H
#define PERCEPTION_JOINT_PERCEPTION_H

#include <boost/shared_ptr.hpp>
#include "Joint.h"
#include "BasicPerception.h"
#include "../action/JointAction.h"


namespace perception {

class JointPerception: public BasicPerception
{
public:
	JointPerception(){};

	~JointPerception(){};

	typedef std::map<unsigned int, Joint> TJointMap;

	Joint& joint( unsigned int jid ) { return mJointMap[jid]; }

	const Joint& operator[]( unsigned int jid ) const {
		TJointMap::const_iterator iter=mJointMap.find(jid);
		return iter->second;
		//return mJointMap[jid]; //TT rewrite for const
	}

    math::AngDeg jointAng( unsigned int jid ) const ;
    math::AngDeg jointRate( unsigned int jid ) const ;

    /**
     * get all angles from the perception
     *
     *
     * @return the map of joint id <--> joint angle
     */
    std::map<unsigned int, math::AngDeg> jointAngles() const;

	const TJointMap& jointMap() const { return mJointMap; }

	bool update( const std::string& msg );

	virtual bool update(const sexp_t* sexp);

	/** update the rate of joints,
	 *  use current angle substract the last joint angle and then div the deltaTime */
	void updateRate( const JointPerception& last, float deltaTime );

	bool updateHJ(const sexp_t* sexp);

	bool updateUJ(const sexp_t* sexp);

	friend std::ostream& operator<<(std::ostream &stream, const JointPerception& p);

	friend std::istream& operator>>(std::istream &stream, JointPerception& p);

	/** exchange pose of left and right foot and hand */
	void exchange();

    /** set all joint angles to zero */
    void zero();

    /**
     * predict the next state of the joints, by the last action
     *
     * @param jact the last action
     * @param t the delta time
     *
     * @return if prediect successfull
     */
    bool predict( boost::shared_ptr<const action::JointAction> jact, float t );


private:

	// mapping from joint id to joint object
    TJointMap mJointMap;

};

std::ostream& operator<<(std::ostream &stream, const JointPerception& p);

std::istream& operator>>(std::istream &stream, JointPerception& p);

} // namespace perception

#endif // PERCEPTION_JOINT_PERCEPTION_H
