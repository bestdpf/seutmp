
#ifndef TASK_KICK_H
#define TASK_KICK_H

#define ENABLE_TASK_KICK_LOG

#include "Task.h"
#include "math/Math.hpp"
#include "perception/JointPerception.h"
#ifdef ENABLE_TASK_KICK_LOG
#include "logger/Logger.h"
#else
#include "logger/NoLogger.h"
#endif

namespace task{
	using namespace boost;
	using namespace math;
    using namespace action;
	
	class KickParameter
	{
		public:
			AngDeg errLength;
			AngDeg errAng;
			Vector2f movePos;
			AngDeg bodyAng;
	};
	
    class Kick: public Task
    {
    public:
		enum KickMode
		{
			KICK_MODE_NORAML = 0,	
			KICK_MODE_FAST, //比较稳定，高度有点高  
			KICK_MODE_LOW,   //左右偏差比LOW2更大点，4度左右，高度似乎比较稳定，高度不高 （很远）
			KICK_MODE_LOW2, //左右有1－2度偏差，高度大多时候可以，高度不高 60m
			KICK_MODE_LOW3,  //高度有时也差距比较大，但是不会高过球门,40m远
			KICK_MODE_HIGH,
			KICK_MODE_PASS_1,  //踢球距离15。5－－－17米
			KICK_MODE_PASS_2,  //踢球距离 大约30－37米，方向不是很稳
			KICK_MODE_PASS_3,   //踢球距离38米，比较稳定
			KICK_MODE_TEST      //20米左右，似乎不错，等待walk完成后调试
		};
		
		struct PoseParameter
		{
			TransMatrixf hip;
			TransMatrixf footL;
			TransMatrixf footR;
		};
		
        /** 
         * create a kick task by kicking target and
         * 
         * @param target target position
 		 * @param mode kick mode
		 * @param isLeftLeg the foot that used
         * @param primary the primary task which create this task
         *
         */
        Kick( const math::Vector2f& target,const KickMode& mode,
				bool isLeftLeg = false,Task* primary = NULL );
		
		void generateDesiredJointAngles(const TransMatrixf& hip,const TransMatrixf& footL,const TransMatrixf& footR,
										perception::JointPerception& desiredPose);
		
		Vector2f calAmendTarget(Vector2f target,Vector2f ballPos,KickMode mode);
		
		void calKickFootLastPos(Vector2f target,Vector2f ballPos,Vector3f& foot);
		
		static bool isGoToRightKickPlace(const Vector2f& target,const Vector2f& ballPos,KickMode mode,bool isLeftLeg);
		
		static bool isGoToRightKickPlace2(const Vector2f& target,
                                          const Vector2f& ballPos,
                                          KickMode mode,
                                          bool isLeftLeg);
		
		float calKickHeight(Vector2f target,Vector2f ballPos,KickMode mode);
		
		static KickParameter calKickParameter(const Vector2f& target,
                                              const Vector2f& ballPos,
                                              KickMode mode,
                                              bool isLeftLeg);

        KickParameter calKickParameter() const;
		
        const Vector2f& getTarget() { return mTarget;}
		
		bool getIsLeftLeg() const { return mIsLeftLeg;}
		
		KickMode getKickMode() const {return mKickMode;}
		
		virtual bool isAchieveable() const;
		
    private:
        /// the target of the kick task
        math::Vector2f mTarget;

        /// the desired driection while reach the target
        KickMode mKickMode;

		bool mIsLeftLeg;
	
		shared_ptr<Action> mActionCache;
	
		perception::JointPerception mDesiredPose;
	
		PoseParameter mPoseParameter;
	
	DECLARE_STATIC_GRAPHIC_LOGGER;
    };
    
} // namespace task

#endif // TASK_KICK_H
