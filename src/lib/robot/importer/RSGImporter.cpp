/***************************************************************************
 *                              SEU RoboCup Simulation Team
 *                     -------------------------------------------------
 * Copyright (c) Southeast University , Nanjing, China.
 * All rights reserved.
 *
 * $Id$
 *
 ****************************************************************************/
#include "RSGImporter.h"
#include <fstream>
#include <typeinfo>

namespace robot{
    namespace importer {

    using namespace parser;
    
    RSGImporter::RSGImporter()
    {
    }

    RSGImporter::~RSGImporter()
    {
    }

    bool RSGImporter::import(const string& filename, shared_ptr< Node<Device> > root)
    {
        TDefinesMap defs;
        sexp_t* args = NULL;
        return import(filename, args, defs, root);
    }

    bool RSGImporter::import(const string& filename,
                             const sexp_t* args,
                             const TDefinesMap& defs,
                             shared_ptr< Node<Device> > node)
    {
        // cout<<__FUNCTION__<<' '<<filename<<endl;
        string content;
        if ( !readFile(filename, content) ){
            return false;
        }
        
        pcont_t* pcont;
 	    sexp_t* sexp;
 	    char* c = const_cast<char*>(content.c_str());
 	    pcont = init_continuation(c);
 	    sexp = iparse_sexp(c,content.size(),pcont);

        if ( !importHeader(sexp) ){
            destroy_sexp(sexp);
            destroy_continuation(pcont);
            return false;
        }

        destroy_sexp(sexp);
        sexp = iparse_sexp(c,content.size(),pcont);
        bool ok = importRSG_0_1(sexp, args, defs, node);
        
        destroy_sexp(sexp);
        destroy_continuation(pcont);
        // cout<<"imported "<<filename<<endl;
        return ok;
    }

    bool RSGImporter::readFile(const string& filename, string& content)
        throw(ClassException<RSGImporter>)
    {
     //   string fullFileName = "/usr/local/share/rcssserver3d/"+filename; //allen comment
        string fullFileName = filename;
        ifstream infile(fullFileName.c_str());
      
        if ( !infile ){
            throw (ClassException<RSGImporter>(" can not find file: "+fullFileName));
            return false;
        }

        while ( !infile.eof() ){
            string line;
            getline(infile, line);
            content += line.substr(0,line.find_first_of(';'));
        }
        
        return true;
    }

    bool RSGImporter::importHeader(const sexp_t* sexp)
        throw(ClassException<RSGImporter>)
    {
        Vector2i version;
        if ( !SexpParser::parseGivenValue(sexp, "RSG", version) ){
            throw( ClassException<RSGImporter>("read the RSG header failed!"));
            return false;
        }

        if ( 0 != version[0] || 1 != version[1] ){
            throw( ClassException<RSGImporter>(" I can not only handle version 0.1!"));
            return false;
        }
        
        return true;
    }

    bool RSGImporter::importRSG_0_1(const sexp_t* sexp,
                                    const sexp_t* args,
                                    const TDefinesMap& defs,
                                    shared_ptr< Node<Device> > node)
        throw(ClassException<RSGImporter>)
    {
        if ( !SexpParser::isList(sexp) ){
            throw( ClassException<RSGImporter>(" read the RSG content failed!"));
            return false;
        }

        TDefinesMap localDefs(defs);
        sexp = sexp->list;
        while( sexp ){

            if ( !SexpParser::isList(sexp) ){
                throw( ClassException<RSGImporter>(" node is not a list!"));
                return false;
            }
            
            const sexp_t* t = sexp->list;
            if ( !parser::SexpParser::isVal(t) ){
                throw( ClassException<RSGImporter>(" can not get what to do!"));
                return false;
            }

            string name(t->val);
            if ( "def" == name ) define(t->next,localDefs);
            else if ( "nd" == name ) importNode(t->next, localDefs, node);
            else if ( "templ" == name ) readTemplate(t->next, args, localDefs);
            else {
                throw( ClassException<RSGImporter>(" I can not handle "+name));
                return false;
            }
            sexp = sexp->next;
        }
        
        return true;
    }

    bool RSGImporter::define(const sexp_t* sexp, TDefinesMap& defs)
        throw(ClassException<RSGImporter>)
    {
        string name;
        if ( !SexpParser::parseValue(sexp, name) ||
             '$' != name[0] )
        {
            throw( ClassException<RSGImporter>(__FUNCTION__+string("  failed!")));
            return false;
        }

        string value;
        calValue(sexp->next, defs, value);
        
        defs[name] = value;
        return true;
    }

    template<typename DATATYPE>
    bool RSGImporter::calValue(const sexp_t* sexp, const TDefinesMap& defs, DATATYPE& value)
        throw(ClassException<RSGImporter>)
    {
        if ( SexpParser::isVal(sexp) )
        {
            if ( '$' == *(sexp->val) )
            {
                string name(sexp->val);
                return findDefs(name, defs, value);
            }
            else
            {
                return SexpParser::parseValue(sexp,value);
            }
        }
        else if ( SexpParser::isList(sexp) )
        {
            sexp = sexp->list;
            string op(sexp->val);
            if( "eval" == op )
            {
                value = lexical_cast<DATATYPE>( eval(sexp->next,defs) );
                return true;
            }
            else if ( "join" == op ){
                value = lexical_cast<DATATYPE>( join(sexp->next,defs) );
                return true;
            }
        
            throw ( ClassException<RSGImporter>("this is not an expression.") );
            return 0;
        }
        else
        {
            throw( ClassException<RSGImporter>("the input is not a value or list!"));
            return false;
        }
    }

    template<typename DATATYPE>
    bool RSGImporter::findDefs(const string& name, const TDefinesMap& defs, DATATYPE& value)
        throw(ClassException<RSGImporter>)
    {
        TDefinesMap::const_iterator v = defs.find(name);
        if ( defs.end() == v )
        {
            throw( ClassException<RSGImporter>(
                       "the variable "+name+" is not defined!"));
            return false;
        }

        if ( '$' == v->second[0] ){
            return findDefs(v->second, defs, value);
        }

        value = lexical_cast<DATATYPE>(v->second);
        return true;
    }
        

    float RSGImporter::eval(const sexp_t* sexp, const TDefinesMap& defs)
        throw(ClassException<RSGImporter>)
    {
        if ( !sexp )
        {
            throw ( ClassException<RSGImporter>("eval missing the first arg.") );
            return 0;
        }
        
        float v;
        if ( !calValue(sexp, defs, v) ) return false;
        const sexp_t* sexpOperator = sexp->next;
        while ( sexpOperator )
        {               
            if ( !parser::SexpParser::isVal(sexpOperator))
            {
                throw ( ClassException<RSGImporter>("unknow eval operator formate.") );
                return 0;
            }
            
            const sexp_t* sexp2 = sexpOperator->next;
            if ( !sexp2 )
            {
                throw ( ClassException<RSGImporter>("eval missing the second arg.") );
                return 0;
            }
            
            float v2;
            if ( !calValue(sexp2, defs, v2) ) return false;
            switch ( *(sexpOperator->val) )
            {
            case '+' : v+=v2;
                break;
            case '-' : v-=v2;
                break;
            case '*' : v*=v2;
                break;
            case '/' : v/=v2;
                break;
            default: throw ( ClassException<RSGImporter>("unknow eval operator.") );
                break;
            }
            sexpOperator = sexp2->next;
        }
        return v;       
    }

    string RSGImporter::join(const sexp_t* sexp, const TDefinesMap& defs)
        throw(ClassException<RSGImporter>)
    {
        string v,tmp;
        while ( sexp )
        {
            calValue(sexp, defs, tmp);
            v += tmp;
            sexp = sexp->next;
        }
        return v;
    }

#define IMPORT_DEVICE(class_name)                           \
    if (#class_name == name ) {                             \
        return importDevice<class_name>(sexp, defs, node);  \
    }

#define CALL_FUNC(function_name)                \
    if (#function_name == func){                \
        dev->function_name();                   \
        return true;                            \
    }
        
#define CALL_STRING_SET_FUNC(function_name)                               \
    if (#function_name == func){                \
        string v; \
        calValue(sexp, defs, v);                  \
        dev->function_name(v);                  \
        return true;                            \
    }


#define CALL_STRING_BOOL_SET_FUNC(function_name)     \
    if (#function_name == func){                \
        string v; \
        bool b;   \
        calValue(sexp, defs, v);                  \
        calValue(sexp->next, defs, b);            \
        dev->function_name(v,b);                   \
        return true;                            \
    }
        
#define CALL_BOOL_SET_FUNC(function_name)                               \
    if (#function_name == func){                                        \
        bool v;                                                         \
        if ( !parser::SexpParser::parseValue(sexp, v) ){                \
            throw ( ClassException<RSGImporter>("unknow "#function_name" value.") ); \
            return false;}                                              \
        dev->function_name(v);                                          \
        return true;                                                    \
    }

#define CALL_FLOAT_SET_FUNC(function_name)      \
    if (#function_name == func){                \
        float v; \
        calValue(sexp, defs, v);                  \
        dev->function_name(v);                  \
        return true;                            \
    }

#define CALL_INT_SET_FUNC(function_name)                                \
    if (#function_name == func){                                        \
        int v;                                                          \
        if ( !calValue(sexp, defs, v) ){                                 \
            throw ( ClassException<RSGImporter>("unknow "#function_name" value.") ); \
            return false;}                                              \
        dev->function_name(v);                                          \
        return true;                                                    \
    }   

#define CALL_INT_FLOAT_SET_FUNC(function_name)                          \
    if (#function_name == func){                                        \
        int i;                                                          \
        if ( !calValue(sexp, defs, i) ){                                 \
            throw ( ClassException<RSGImporter>("unknow "#function_name" value.") ); \
            return false;}                                              \
        float v;                                                        \
        calValue(sexp->next, defs, v);                                  \
        dev->function_name(i,v);                                        \
        return true;                                                    \
    }   

#define CALL_FLOAT_FLOAT_SET_FUNC(function_name)    \
    if (#function_name == func){                    \
        float a,b;                                  \
        calValue(sexp, defs, a);                    \
        calValue(sexp->next, defs, b);              \
        dev->function_name(a,b);                    \
        return true;                                \
    }
	
#define CALL_FLOAT_FLOAT_FLOAT_SET_FUNC(function_name) \
	if(#function_name == func)	\
	{									\
		float a,b,c;					\
		calValue(sexp, defs, a);	\
		calValue(sexp->next, defs, b);		\
		calValue(sexp->next, defs, c);		\
		dev->function_name(a, b, c);		\
		return true;								\
	}												\
    
#define CALL_VECTOR3F_SET_FUNC(function_name )                          \
    if (#function_name == func ){                                       \
        Vector3f pos;                                                   \
        if ( !calVector3f(sexp, defs, pos) ){                           \
            throw ( ClassException<RSGImporter>("unknow "#function_name" value.") ); \
            return false;                                               \
        }                                                               \
        dev->function_name(pos);                                        \
        return true;                                                    \
    }

#define CALL_FLOAT_VECTOR3F_SET_FUNC(function_name )                    \
    if (#function_name == func ){                                       \
        float f;                                                        \
        calValue(sexp,defs, f);                                         \
        Vector3f v;                                                     \
        if ( !calVector3f(sexp->next, defs, v) ){                       \
            throw ( ClassException<RSGImporter>("unknow "#function_name" value.") ); \
            return false;                                               \
        }                                                               \
        dev->function_name(f,v);                                        \
        return true;                                                    \
    }

#define CALL_PARENT_METHODS(class_name)                                 \
    shared_ptr<class_name> p = shared_static_cast<class_name>( dev );   \
    return invoke(p, func, sexp, defs);

#define DEFINE_DEVICE_INVOKE(class_name)                    \
    bool RSGImporter::invoke(shared_ptr<class_name> dev,    \
                             const string& func,            \
                             const sexp_t* sexp,            \
                             const TDefinesMap& defs)

    bool RSGImporter::importNode(const sexp_t* sexp,
                                 const TDefinesMap& defs,
                                 shared_ptr< Node<Device> > node)
        throw(ClassException<RSGImporter>)
    {
        if ( !SexpParser::isVal(sexp) ){
            throw ( ClassException<RSGImporter>("unknow node name formate.") );
            return false;
        }
        
        string name(sexp->val);
        // cout<<__FUNCTION__<<' '<<name<<endl;
        sexp = sexp->next;
        if ( "Space" == name ){
            return importDevice<Space>( sexp, defs, node );
        }
        if ( "SingleMatNode" == name ){
            // skip texture
            return true;
        }
        if ( "Capsule" == name ){
            // skip: the Capsule is just for looking
            return true;
        }
        if ( "Box" == name ){
            // skip: the Box is just for looking
            return true;
        }
        if ( "Sphere" == name ){
            // skip: the Sphere is just for looking
            return true;
        }
        if ( "TouchPerceptorHandler" == name ){
            // skip: this is only need by server
            return true;
        }
        IMPORT_DEVICE(Transform);
        IMPORT_DEVICE(AgentAspect);
        IMPORT_DEVICE(AgentState);
        IMPORT_DEVICE(InitEffector);
        IMPORT_DEVICE(SingleMatInitEffector);
        IMPORT_DEVICE(StaticMeshInitEffector);
        IMPORT_DEVICE(BeamEffector);
        IMPORT_DEVICE(SayEffector);
        ////////////////////////////////////////////////////////////////////
        IMPORT_DEVICE(AgentSyncEffector);
        /////////////////////////////////////////////////////
        IMPORT_DEVICE(UniversalJointEffector);
        if ( "HingeEffector" == name ){
            return importDevice<HingeJointEffector>(sexp, defs, node);
        }

        IMPORT_DEVICE(RigidBody);
        IMPORT_DEVICE(DragController);
        IMPORT_DEVICE(Cylinder);
        IMPORT_DEVICE(BoxCollider);
        IMPORT_DEVICE(SphereCollider);
        IMPORT_DEVICE(CapsuleCollider);
        IMPORT_DEVICE(TransformCollider);
        IMPORT_DEVICE(ContactJointHandler);
        IMPORT_DEVICE(HingeJoint);
        IMPORT_DEVICE(FixedJoint);
        IMPORT_DEVICE(UniversalJoint);
        if ( "TimePerceptor" == name ){
            return importDevice<TimeSensor>(sexp, defs, node );
        }
        if ( "GameStatePerceptor" == name ){
            return importDevice<GameStateSensor>(sexp, defs, node );
        }
        if ( "HearPerceptor" == name ){
            return importDevice<HearSensor>(sexp, defs, node );
        }
        if ( "GyroRatePerceptor" == name ){
            return importDevice<GyroRateSensor>(sexp, defs, node );
        }
        //////////////////////////////////add by allen
        if ( "Accelerometer" == name ){
            return importDevice<AccelerometerSensor>(sexp, defs, node );
        }
        ///////////////////////////////////////////////
        if ( "VisionPerceptor" == name ){
            return importDevice<VisionSensor>(sexp, defs, node );
        }
        if ( "RestrictedVisionPerceptor" == name ){
            return importDevice<RestrictedVisionSensor>(sexp, defs, node);
        }
        if ( "UniversalJointPerceptor" == name ){
            return importDevice<UniversalJointSensor>(sexp, defs, node);
        }
        if ( "HingePerceptor" == name ){
            return importDevice<HingeJointSensor>(sexp, defs, node);
        }
        if ( "ForceResistancePerceptor" == name ){
            return importDevice<ForceResistanceSensor>(sexp, defs, node);
        }
        IMPORT_DEVICE(Camera);
        IMPORT_DEVICE(ObjectState);
        throw ( ClassException<RSGImporter>("can not create "+name) );
        return false;
    }

    template <typename DEVICE>
    bool RSGImporter::importDevice(const sexp_t* sexp,
                                   TDefinesMap defs,
                                   shared_ptr< Node<Device> > node)
        throw(ClassException<RSGImporter>)
    {
        shared_ptr< DEVICE > dev ( new DEVICE );
        shared_ptr< Node<Device> > child( Node<Device>::create( dev ) );
        node->addChild( child );
        mCurrentNode = child;
        
        while( sexp ){
            if ( !SexpParser::isList(sexp) ){
                throw ( ClassException<RSGImporter>("unknow formate for class Device.") );
                return false;
            }
            
            const sexp_t* t = sexp->list;
            string funName;
            if ( !SexpParser::parseValue(t, funName) ){
                throw ( ClassException<RSGImporter>("unknow function formate for class Device.") );
                return false;
            }
            else if ( "nd" == funName ){
                if ( !importNode(t->next, defs, child) )
                    return false;
            }
            else if ( "importScene" == funName ){
                if ( !importScene(t->next, defs, child) )
                    return false;
            }
            else if ( "def" == funName ) {
                if ( !define(t->next, defs) )
                    return false;
            }
            //else if ( "switch" == funName ){
                // TODO: just skip it now, since it only relative to textures
                //string val;
                //calValue(sexp, defs, val);
                //return true;
            //}
            else{
                if ( !invoke(dev, funName, t->next, defs) ){
                    throw (ClassException<RSGImporter>("unknow "
                                                       +string(typeid(*(dev.get())).name())
                                                       +" function: "+funName));
                    return false;
                }
            }
            sexp = sexp->next;
        }
        return true;
    }

    DEFINE_DEVICE_INVOKE(Device)
    {
        if ( "switch" == func ){
            // TODO: should not be here, see importDevice
            return true;
        }
        if ( "setName" == func ){
            string val;
            calValue(sexp, defs, val);
            dev->setName(val);
            return true;
        }

        return false;
    }

    DEFINE_DEVICE_INVOKE(Space)
    {
        CALL_BOOL_SET_FUNC(disableInnerCollision);
        
        CALL_PARENT_METHODS(Device);
    }

    DEFINE_DEVICE_INVOKE(Transform)
    {
        CALL_VECTOR3F_SET_FUNC(setLocalPos);
        CALL_VECTOR3F_SET_FUNC(setLocalRotation);

        CALL_PARENT_METHODS(Device);
    }

    DEFINE_DEVICE_INVOKE(AgentAspect)
    {
        CALL_PARENT_METHODS(Transform);
    }

    DEFINE_DEVICE_INVOKE(AgentState)
    {
        CALL_PARENT_METHODS(Device);
    }

     DEFINE_DEVICE_INVOKE(Cylinder)
    {
        CALL_FLOAT_FLOAT_SET_FUNC(setParams);
        CALL_FLOAT_FLOAT_FLOAT_SET_FUNC(setScale);
        CALL_STRING_SET_FUNC(setMaterial);
        CALL_FUNC(setTransparent);
        CALL_PARENT_METHODS(Device);
    }

    DEFINE_DEVICE_INVOKE(RigidBody)
    {
        CALL_FLOAT_VECTOR3F_SET_FUNC(setBox);
        CALL_FLOAT_VECTOR3F_SET_FUNC(setBoxTotal);
        CALL_FLOAT_FLOAT_SET_FUNC(setSphere);
        CALL_FLOAT_FLOAT_SET_FUNC(setSphereTotal);
        CALL_FLOAT_FLOAT_FLOAT_SET_FUNC(setCapsuleTotal);
        if ( "addBox" == func ){
            float mass;
            Vector3f size,p,rot;
            calValue(sexp, defs, mass);
            sexp = sexp->next;
            if ( !calVector3f(sexp, defs, size) ) return false;
            for( int i=0; i<3; i++){
                sexp = sexp->next;
                if ( NULL == sexp ) return false;
            }
            if ( !calVector3f(sexp, defs, p) ) return false;
            for( int i=0; i<3; i++){
                sexp = sexp->next;
                if ( NULL == sexp ) return false;
            }
            if ( !calVector3f(sexp, defs, rot) ) return false;
            dev->addBox(mass,size,p,rot);
            return true;
        }
        
        CALL_PARENT_METHODS(Device);
    }
    
    bool RSGImporter::importScene(const sexp_t* sexp,
                                  const TDefinesMap& defs,
                                  shared_ptr< Node<Device> > node)
        throw ( ClassException<RSGImporter> )
    {
        string fileName;
        if ( !SexpParser::parseValue(sexp,fileName) ){
            throw ( ClassException<RSGImporter>("unknow importScene name filename.") );
            return false;
        }

        return import(fileName, sexp->next, defs, node);
    }

    bool RSGImporter::calVector3f(const sexp_t* sexp,
                                  const TDefinesMap& defs,
                                  Vector3f& p)
        throw(ClassException<RSGImporter>)
    {
        for( int i=0 ;i<3; i++ ){
            if ( !sexp ){
                throw ( ClassException<RSGImporter>("unknow Vector3f formate.") );
                return false;
            }
            
            if ( !calValue(sexp,defs,p[i]) ) return false;
            sexp = sexp->next;
        }
        return true;
    }

    bool RSGImporter::readTemplate( const sexp_t* sexp,
                                    const sexp_t* args,
                                    TDefinesMap& defs)
        throw(ClassException<RSGImporter>)
    {
        while ( sexp )
        {
            if ( !args ){
                throw ( ClassException<RSGImporter>("the arguments number is different.") );
                return false;
            }
            
            if (!SexpParser::isVal(sexp)){
                throw ( ClassException<RSGImporter>("unknow template formate.") );
                return false;
            }

            string tmp;
            calValue(args, defs, tmp);
            defs[string(sexp->val)] =  tmp;
            
            sexp = sexp->next;
            args = args->next;
        }
        return true;
    }

    DEFINE_DEVICE_INVOKE(DragController)
    {
        CALL_FLOAT_SET_FUNC(setAngularDrag);
        CALL_FLOAT_SET_FUNC(setLinearDrag);
        
        CALL_PARENT_METHODS(Device);
    }
    
    DEFINE_DEVICE_INVOKE(Collider)
    {
        CALL_STRING_BOOL_SET_FUNC(addNotCollideWithColliderName);
        CALL_VECTOR3F_SET_FUNC(setLocalPosition);
        CALL_VECTOR3F_SET_FUNC(setRotation);
        
        CALL_PARENT_METHODS(Device);
    }
    
    DEFINE_DEVICE_INVOKE(BoxCollider)
    {
        if ( "setBoxLengths" == func ){
            Vector3f len;
            if ( ! ( calVector3f(sexp, defs, len) )) {
                throw ( ClassException<RSGImporter>("unknow setBoxLengths value.") );
                return false;
            }
            dev->setBoxLengths(len);
            return true;
        }

        CALL_PARENT_METHODS(Collider);
    }

    DEFINE_DEVICE_INVOKE(SphereCollider)
    {
        CALL_FLOAT_SET_FUNC(setRadius);

        CALL_PARENT_METHODS(Collider);
    }

    DEFINE_DEVICE_INVOKE(CapsuleCollider)
    {
        CALL_FLOAT_FLOAT_SET_FUNC(setParams);

        CALL_PARENT_METHODS(Collider);
    }

    DEFINE_DEVICE_INVOKE(TransformCollider)
    {
        CALL_PARENT_METHODS(Collider);
    }

    DEFINE_DEVICE_INVOKE(CollisionHandler)
    {
        CALL_PARENT_METHODS(Device);
    }

    DEFINE_DEVICE_INVOKE(ContactJointHandler)
    {
        CALL_BOOL_SET_FUNC(setContactBounceMode);
        CALL_FLOAT_SET_FUNC(setContactBounceValue);
        CALL_FLOAT_SET_FUNC(setMinBounceVel);
        CALL_BOOL_SET_FUNC(setContactSlipMode);
        CALL_FLOAT_SET_FUNC(setContactSlip);
        CALL_BOOL_SET_FUNC (setContactSoftERPMode);
        CALL_FLOAT_SET_FUNC(setContactSoftERP);
        CALL_BOOL_SET_FUNC(setContactSoftCFMMode);
        CALL_FLOAT_SET_FUNC(setContactSoftCFM);
        
        CALL_PARENT_METHODS(CollisionHandler);
    }

    DEFINE_DEVICE_INVOKE(InitEffector)
    {
        CALL_PARENT_METHODS(Effector);
    }

    DEFINE_DEVICE_INVOKE(SingleMatInitEffector)
    {
        CALL_PARENT_METHODS(InitEffector);
    }

    DEFINE_DEVICE_INVOKE(StaticMeshInitEffector)
    {
        CALL_PARENT_METHODS(InitEffector);
    }

    DEFINE_DEVICE_INVOKE(Effector)
    {
        CALL_PARENT_METHODS(Device);
    }

    DEFINE_DEVICE_INVOKE(BeamEffector)
    {
        CALL_PARENT_METHODS(Effector);
    }

    DEFINE_DEVICE_INVOKE(SayEffector)
    {
        CALL_PARENT_METHODS(Effector);
    }
////////////////////////////////////////////////////////
     DEFINE_DEVICE_INVOKE(AgentSyncEffector)
    {
        CALL_PARENT_METHODS(Effector);
    }
     //////////////////////////////////////
    DEFINE_DEVICE_INVOKE(UniversalJointEffector)
    {
        CALL_PARENT_METHODS(Effector);
    }

    DEFINE_DEVICE_INVOKE(HingeJointEffector)
    {
        CALL_PARENT_METHODS(Effector);
    }

    DEFINE_DEVICE_INVOKE(TimeSensor)
    {
        CALL_PARENT_METHODS(Sensor);
    }

    DEFINE_DEVICE_INVOKE(Sensor)
    {
        CALL_INT_SET_FUNC(setInterval);
        
        CALL_PARENT_METHODS(Device);
    }

    DEFINE_DEVICE_INVOKE(GameStateSensor)
    {
        CALL_PARENT_METHODS(Sensor);
    }

    DEFINE_DEVICE_INVOKE(HearSensor)
    {
        CALL_PARENT_METHODS(Sensor);
    }

    DEFINE_DEVICE_INVOKE(GyroRateSensor)
    {
        CALL_PARENT_METHODS(Sensor);
    }
    //////////////////////////////////add by allen
    DEFINE_DEVICE_INVOKE(AccelerometerSensor)
    {
        CALL_PARENT_METHODS(Sensor);
    }
    //////////////////////////////////////
    DEFINE_DEVICE_INVOKE(UniversalJointSensor)
    {
        CALL_PARENT_METHODS(Sensor);
    }

    DEFINE_DEVICE_INVOKE(HingeJointSensor)
    {
        CALL_PARENT_METHODS(Sensor);
    }
    
    DEFINE_DEVICE_INVOKE(VisionSensor)
    {
        CALL_BOOL_SET_FUNC(setSenseMyPos);
        CALL_BOOL_SET_FUNC(setStaticSenseAxis);
        CALL_BOOL_SET_FUNC(addNoise);

        CALL_PARENT_METHODS(Sensor);
    }

    DEFINE_DEVICE_INVOKE(RestrictedVisionSensor)
    {
        CALL_FLOAT_FLOAT_SET_FUNC(setViewCones);
        CALL_BOOL_SET_FUNC(setSenseMyTrans);
        CALL_PARENT_METHODS(VisionSensor);
    }

    DEFINE_DEVICE_INVOKE(ForceResistanceSensor)
    {   
        CALL_PARENT_METHODS(Sensor);
    }

    DEFINE_DEVICE_INVOKE(Camera)
    {
        CALL_PARENT_METHODS(Sensor);
    }
    
    DEFINE_DEVICE_INVOKE(Joint)
    {
        if ( "attach" == func ){
            string body1path, body2path;
            if ( !(calValue(sexp, defs, body1path)
                   &&calValue(sexp->next, defs, body2path))){
                throw ClassException<RSGImporter>("unknow attach value");
                return false;
            }
            
            shared_ptr<Device> dev1 = mCurrentNode->findByPath(body1path)->data();
            shared_ptr<Device> dev2 = mCurrentNode->findByPath(body2path)->data();
            shared_ptr<RigidBody> body1 = shared_dynamic_cast<RigidBody>(dev1);
            if ( NULL == body1.get() ){
                throw ClassException<RSGImporter>("can not find body: "+body1path);
                return false;
            }
            shared_ptr<RigidBody> body2 = shared_dynamic_cast<RigidBody>(dev2);
            if ( NULL == body2.get() ){
                throw ClassException<RSGImporter>("can not find body: "+body2path);
                return false;
            }
            dev->attach(body1, body2);
            return true;
        }
        CALL_VECTOR3F_SET_FUNC(setAnchor);
        CALL_INT_FLOAT_SET_FUNC(setMaxMotorForce);
        CALL_INT_FLOAT_SET_FUNC(setLowStopDeg);
        CALL_INT_FLOAT_SET_FUNC(setHighStopDeg);
        CALL_INT_FLOAT_SET_FUNC(setCFM);
        CALL_INT_FLOAT_SET_FUNC(setStopCFM);
        CALL_INT_FLOAT_SET_FUNC(setStopERP);
        CALL_INT_FLOAT_SET_FUNC(setFudgeFactor);
        CALL_INT_FLOAT_SET_FUNC(setBounce);
        CALL_FLOAT_SET_FUNC(setJointMaxSpeed1);
        CALL_FLOAT_SET_FUNC(setJointMaxSpeed2);

        CALL_PARENT_METHODS(Device);
    }
    
    DEFINE_DEVICE_INVOKE(HingeJoint)
    {
        if ( "setAxis" == func ){
            Vector3f axis;
            int a;
            if ( NULL != sexp->next && calVector3f(sexp, defs, axis) ){
                dev->setAxis(axis);
                return true;
            }
            else if ( calValue(sexp, defs, a) ) {
                dev->setAxis(a);
                return true;
            }
            else{
                return false;
            }
        }
        
        CALL_PARENT_METHODS(Joint);
    }

    DEFINE_DEVICE_INVOKE(FixedJoint)
    {
        CALL_FUNC(setFixed);
        
        CALL_PARENT_METHODS(Joint);
    }

    DEFINE_DEVICE_INVOKE(UniversalJoint)
    {
        CALL_VECTOR3F_SET_FUNC(setAxis1);
        CALL_VECTOR3F_SET_FUNC(setAxis2);
        
        CALL_PARENT_METHODS(Joint);
    }

    DEFINE_DEVICE_INVOKE(ObjectState)
    {
        CALL_STRING_SET_FUNC(setID);
        
        CALL_PARENT_METHODS(Device);
    }
        
    } /* namespace rsgimporter */
} /* namespace robot */
