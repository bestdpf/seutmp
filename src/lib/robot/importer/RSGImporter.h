/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#ifndef _RSGIMPORTER_RSGIMPORTER_H_
#define _RSGIMPORTER_RSGIMPORTER_H_

#include "Node.hpp"
#include "../device/Device.h"
#include "ClassException.hpp"
#include "parser/SexpParser.hpp"
#include "../device/Space.h"
#include "../device/AgentAspect.h"
#include "../device/AgentState.h"
#include "../device/RigidBody.h"
#include "../device/DragController.h"
#include "../device/Cylinder.h"
#include "../device/collider/BoxCollider.h"
#include "../device/collider/SphereCollider.h"
#include "../device/collider/CapsuleCollider.h"
#include "../device/collider/ContactJointHandler.h"
#include "../device/collider/TransformCollider.h"
#include "../device/effector/InitEffector.h"
#include "../device/effector/BeamEffector.h"
#include "../device/effector/SayEffector.h"
#include "../device/effector/AgentSyncEffector.h"
#include "../device/effector/UniversalJointEffector.h"
#include "../device/effector/HingeJointEffector.h"
#include "../device/sensor/TimeSensor.h"
#include "../device/sensor/GameStateSensor.h"
#include "../device/sensor/HearSensor.h"
#include "../device/sensor/UniversalJointSensor.h"
#include "../device/sensor/HingeJointSensor.h"
#include "../device/sensor/GyroRateSensor.h"
#include "../device/sensor/AccelerometerSensor.h"////////////////////////
#include "../device/sensor/VisionSensor.h"
#include "../device/sensor/ForceResistanceSensor.h"
#include "../device/sensor/Camera.h"
#include "../device/joint/HingeJoint.h"
#include "../device/joint/FixedJoint.h"
#include "../device/joint/UniversalJoint.h"
#include "../device/ObjectState.h"

namespace robot{
    namespace importer {

    using namespace std;
    using namespace boost;
    using namespace tree;
    using namespace robot::device;
    using namespace robot::device::collider;
    using namespace robot::device::effector;
    using namespace robot::device::sensor;
    using namespace robot::device::joint;
    
    class RSGImporter 
    {
        /// the map for defines in the file
        typedef map<string, string> TDefinesMap;
        
    public:
        RSGImporter();
        
        virtual ~RSGImporter();

        /** 
         * import the scene from a given file
         * 
         * @param filename the configuration file
         * @param node the root node
         * 
         * @return if the importing successful
         */
        bool import(const string& filename, shared_ptr< Node<Device> > root);
        
    private:
        /** 
         * import the Ruby Scene Graphic from a given file
         * 
         * @param filename the Ruby Scene Grahpic file
         * @param args the arguments passed
         * @param defs current defined variables
         * @param node the parent node
         * 
         * @return if the importing successful
         */
        bool import(const string& filename,
                    const sexp_t* args,
                    const TDefinesMap& defs,
                    shared_ptr< Node<Device> > node);
        
        /** 
         * read the content from a given filename, the comments will
         * be ignored
         * 
         * @param filename the filename
         * @param content the result content
         * 
         * @return if reading successful
         */
        bool readFile(const string& filename, string& content)
            throw(ClassException<RSGImporter>);

        /** 
         * import the header of Ruby Scene Graphic, it contains
         * version number. Currently only "(RSG 0 1)" is supported
         * 
         * @param sexp the S expression strut
         * 
         * @return if the header is imported successfully
         */
        bool importHeader(const sexp_t* sexp)
            throw(ClassException<RSGImporter>);

        /** 
         * import the content, handle RSG version 0.1
         * 
         * @param sexp the S expression strut
         * @param args the arguments passed
         * @param defs current defined variables
         * @param node the parent node
         * 
         * @return 
         */
        bool importRSG_0_1(const sexp_t* sexp,
                           const sexp_t* args,
                           const TDefinesMap& defs,
                           shared_ptr< Node<Device> > node)
            throw(ClassException<RSGImporter>);

        /** 
         * parse the define sentence
         * 
         * @param sexp the input
         * @param defs the current stack
         * 
         * @return if the input is well formated
         */
        bool define(const sexp_t* sexp, TDefinesMap& defs)
            throw(ClassException<RSGImporter>);

        /** 
         * calculate the value of a given S expression:
         * - if it is a list, try to evaluate it
         * - else if it start with `$', find it in the defines
         * - else it should be a value
         * 
         * @param sexp the input S expression
         * @param defs the current defines
         * 
         * @return the value
         */
        template<typename DATATYPE>
        bool calValue(const sexp_t* sexp, const TDefinesMap& defs, DATATYPE& value)
            throw(ClassException<RSGImporter>);

        template<typename DATATYPE>
        bool findDefs(const string& name, const TDefinesMap& defs, DATATYPE& value)
            throw(ClassException<RSGImporter>);
        
        bool calVector3f(const sexp_t* sexp,const TDefinesMap& defs,Vector3f& p)
            throw(ClassException<RSGImporter>);
        
        /** 
         * evaluate the math expression
         * 
         * @param sexp the input S expression
         * @param defs the current defines
         * 
         * @return the results value
         */
        float eval(const sexp_t* sexp, const TDefinesMap& defs)
            throw(ClassException<RSGImporter>);

        /** 
         * join the expression as a string
         * 
         * @param sexp the list
         * @param defs the defines
         * 
         * @return the joined string
         */
        string join(const sexp_t* sexp, const TDefinesMap& defs)
            throw(ClassException<RSGImporter>);

        /** 
         * import a node, it usually create a new device
         * 
         * @param sexp the input
         * @param defs the current defines
         * @param node the parent node, i.e. the new node will be
         * attached as its child
         * 
         * @return if the node is created successfully
         */
        bool importNode(const sexp_t* sexp,
                        const TDefinesMap& defs,
                        shared_ptr< Node<Device> > node)
            throw(ClassException<RSGImporter>);

        bool importScene(const sexp_t* sexp,
                         const TDefinesMap& defs,
                         shared_ptr< Node<Device> > node)
            throw(ClassException<RSGImporter>);

        bool readTemplate( const sexp_t* sexp,
                           const sexp_t* args,
                           TDefinesMap& defs)
            throw(ClassException<RSGImporter>);
        
        /** 
         * a template to import virtual device, usually it do not have
         * special function
         * 
         * @param sexp the input
         * @param defs the current defs
         * @param node the parent node
         * 
         * @return if the device is created successfully
         */
        template <typename DEVICE>
        bool importDevice(const sexp_t* sexp,
                          TDefinesMap defs,
                          shared_ptr< Node<Device> > node)
            throw(ClassException<RSGImporter>);

#define DECLARE_DEVICE_INVOKE(class_name)       \
        bool invoke(shared_ptr<class_name> dev, \
                    const string& func,         \
                    const sexp_t* sexp,         \
                    const TDefinesMap& defs)
        
        DECLARE_DEVICE_INVOKE(Device);
        DECLARE_DEVICE_INVOKE(Space);
        DECLARE_DEVICE_INVOKE(Transform);
        DECLARE_DEVICE_INVOKE(AgentAspect);
        DECLARE_DEVICE_INVOKE(AgentState);
        DECLARE_DEVICE_INVOKE(RigidBody);
        DECLARE_DEVICE_INVOKE(DragController);
        DECLARE_DEVICE_INVOKE(Cylinder);
        DECLARE_DEVICE_INVOKE(Collider);
        DECLARE_DEVICE_INVOKE(BoxCollider);
        DECLARE_DEVICE_INVOKE(SphereCollider);
        DECLARE_DEVICE_INVOKE(CapsuleCollider);
        DECLARE_DEVICE_INVOKE(TransformCollider);
        DECLARE_DEVICE_INVOKE(CollisionHandler);
        DECLARE_DEVICE_INVOKE(ContactJointHandler);
        DECLARE_DEVICE_INVOKE(Effector);
        DECLARE_DEVICE_INVOKE(InitEffector);
        DECLARE_DEVICE_INVOKE(SingleMatInitEffector);
        DECLARE_DEVICE_INVOKE(StaticMeshInitEffector);
        DECLARE_DEVICE_INVOKE(BeamEffector);
        DECLARE_DEVICE_INVOKE(SayEffector);
        DECLARE_DEVICE_INVOKE(AgentSyncEffector);////////////////////////////
        DECLARE_DEVICE_INVOKE(UniversalJointEffector);
        DECLARE_DEVICE_INVOKE(HingeJointEffector);
        DECLARE_DEVICE_INVOKE(Sensor);
        DECLARE_DEVICE_INVOKE(TimeSensor);
        DECLARE_DEVICE_INVOKE(GameStateSensor);
        DECLARE_DEVICE_INVOKE(HearSensor);
        DECLARE_DEVICE_INVOKE(UniversalJointSensor);
        DECLARE_DEVICE_INVOKE(HingeJointSensor);
        DECLARE_DEVICE_INVOKE(GyroRateSensor);
        DECLARE_DEVICE_INVOKE(AccelerometerSensor);/////////////////////////
        DECLARE_DEVICE_INVOKE(VisionSensor);
        DECLARE_DEVICE_INVOKE(RestrictedVisionSensor);
        DECLARE_DEVICE_INVOKE(ForceResistanceSensor);
        DECLARE_DEVICE_INVOKE(Camera);
        DECLARE_DEVICE_INVOKE(Joint);
        DECLARE_DEVICE_INVOKE(HingeJoint);
        DECLARE_DEVICE_INVOKE(FixedJoint);
        DECLARE_DEVICE_INVOKE(UniversalJoint);
        DECLARE_DEVICE_INVOKE(ObjectState);

    private:
        shared_ptr< Node<Device> > mCurrentNode;
    };
        
    } /* namespace importer */
} /* namespace robot */

#endif /* _RSGIMPORTER_RSGIMPORTER_H_ */
