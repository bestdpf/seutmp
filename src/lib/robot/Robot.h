/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _ROBOT_ROBOT_H_
#define _ROBOT_ROBOT_H_

#include "device/Device.h"
#include "Node.hpp"

namespace robot {
    using namespace std;
    using namespace boost;
    using namespace tree;
    using namespace device;
    
    class Robot
    {
    public:
        Robot(const string& name);
        
        virtual ~Robot();

        const string& getName() const
            {
                return mName;
            }

        /** 
         * get the reference to the Singleton
         * 
         * 
         * @return the reference
         */
        static Robot& getSingleton() { return *mTheRobot; }

        /** 
         * initialize the robot, i.e set the Singleton
         * 
         * @param r the pointer to the robot
         */
        template <typename ROBOT_TYPE>
        static bool init()
            {
                destrut();
                mTheRobot = new ROBOT_TYPE();
                return NULL != mTheRobot;
            }

        /** 
         * release the Singleton
         * 
         */
        static void destrut();
        
        shared_ptr<const Node<Device> > getDevices() const
            {
                return mDevice;
            }

        /** 
         * get all the devices of DEVICE of the robot
         * 
         * @param results 
         */
        template <typename DEVICE>
        void listAll(list<shared_ptr<const Node<Device> > >& results) const
            {
                mDevice->listChildren<DEVICE>( results, true );
            }

        template <typename DEVICE>
        void listAll(list<shared_ptr<Node<Device> > >& results)
            {
                mDevice->listChildren<DEVICE>( results, true );
            }

    protected:
        static Robot* mTheRobot;
        
    private:
        
        /** 
         * this function update the global matrix of transform,
         * according to the devices tree, because the transform only
         * be set by local matrix
         * 
         * @param dev the devices' tree
         * @param mat the mother's global matrix
         */
        void updateTransformGlobalMat(shared_ptr<Node<Device> > dev,
                                      TransMatrixf mat) const;

        /** assemble the robot according to name */
        void assemble();
        
        /** the device root of this robot */
        shared_ptr< Node<Device> > mDevice;
        
        /** the name of robot */
        string mName;
    };

#define ROBOT robot::Robot::getSingleton()
    
} /* namespace robot */

#endif /* _ROBOT_ROBOT_H_ */
