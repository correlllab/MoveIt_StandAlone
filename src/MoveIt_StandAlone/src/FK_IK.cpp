#include "FK_IK.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

/*************** Class Method Definitions ***************/

FK_IK_Service::FK_IK_Service( ros::NodeHandle& _nh ){
    ROS_INFO("FK/IK Node starting ...");
    _nh = _nh;
    pathsOK = load_JSON_config();
    setup_FK();
    init_services();
}

bool FK_IK_Service::load_JSON_config(){
    Json::Reader reader;
    Json::Value  obj;
    string /*-*/ pkgPath = ros::package::getPath( _PKG_NAME ) ;
    string /*-*/ configPath = pkgPath + "/config/config.json";
    bool /* - */ foundConfig = check_exist( configPath , "file" );

    if( foundConfig ){
        ROS_INFO( "Found the config file: %s" , configPath.c_str() );

        cout << "About to read file ..." << endl;

        ifstream ifs( configPath );
        reader.parse( ifs , obj ); // reader can also read strings

        cout << "Building paths ..." << endl;

        // Topics
        FK_srv_topicName = obj[ "UR_FKservice_TOPIC"  ].asString();
        IK_srv_topicName = obj[ "UR_IKservice_TOPIC"  ].asString();

        // Robot Desc. Paths
        URDF_full_path = pkgPath + "/" + obj[ "URDF"  ].asString();
        SRDF_full_path = pkgPath + "/" + obj[ "SRDF"  ].asString();

        // Robot Description
        KDL_joint_group_name = obj[ "FK_joint_group_name" ].asString();
        end_link_name /*--*/ = obj[ "end_link" ].asString();

        ifs.close();

        cout << "Check existence ..." << endl;

        bool foundURDF = check_exist( URDF_full_path , "file" );
        if( !foundURDF ){
            ROS_ERROR( "Could NOT locate URDF file: %s" , URDF_full_path.c_str() );
        }
        bool foundSRDF = check_exist( SRDF_full_path , "file" );
        if( !foundSRDF ){
            ROS_ERROR( "Could NOT locate SRDF file: %s" , SRDF_full_path.c_str() );
        }

        if( foundURDF && foundSRDF )
            ROS_INFO( "Files OK!" );

        return foundURDF && foundSRDF;
    }else{
        ROS_ERROR( "Could NOT locate config file: %s" , configPath.c_str() );
        return false;
    }   
}

bool FK_IK_Service::setup_FK(){
    if( pathsOK ){
        
        // 1. Load *RDF
        string URDF = get_file_string( URDF_full_path );
        string SRDF = get_file_string( SRDF_full_path );

        // ROS complains if these are not set
        _nh.setParam( "/robot_description"          , URDF );
        _nh.setParam( "/robot_description_semantic" , SRDF );

        // 2. Create robot model
        opt /*-----------*/ = robot_model_loader::RobotModelLoader::Options( URDF , SRDF );
        robot_model /*---*/ = robot_model_loader::RobotModelLoader( opt );
        kinematic_model     = robot_model.getModel();
        kinematic_state_ptr = robot_state::RobotStatePtr( new robot_state::RobotState( kinematic_model ) );

        // 3. Get a pointer to the joint group
        joint_model_group_ptr = kinematic_model->getJointModelGroup( KDL_joint_group_name );
        if( !joint_model_group_ptr ){
            ROS_ERROR( "Could not load joint model group: %s" , KDL_joint_group_name.c_str() );
            return false;
        }else
            ROS_INFO( "Kinematic model OK!" );

        const std::vector<std::string> &joint_names = joint_model_group_ptr->getJointModelNames();
        N_joints = joint_names.size() - 1;
        ROS_INFO( "Kinematic chain has %u joints" , N_joints );

        return true;

    }else{
        ROS_ERROR( "Could not start FK service due to incorrect paths. \nURDF: %s \nSRDF: %s" , URDF_full_path.c_str() , SRDF_full_path.c_str() );
    }
}

bool FK_IK_Service::init_services(){
    // string servName = "serv" ;
    FKservice = _nh.advertiseService( FK_srv_topicName , &FK_IK_Service::FK_cb , this );
}

bool FK_IK_Service::load_q( const ur_motion_planning::FK_req& q ){
    bool _DEBUG = true;

    if( _DEBUG ){
        cout << "Setting the joint model!" << endl;
        cout << "Got a vector with " << q.q_joints.size() << " elements." << endl;
    }  

    std::vector<double> joint_values;
    double val = -50.0;
    for( u_char i = 0 ; i < N_joints ; i++ ){
       val = q.q_joints[i];
       if( _DEBUG )  cout << "Joint " << i+1 << " of " << (int) N_joints << ", Value: " << val << endl;
       joint_values.push_back( val );
    }
    if( _DEBUG )  cout << "Retreived the joint values!" << endl;

    kinematic_state_ptr->setJointGroupPositions( joint_model_group_ptr , joint_values );

    if( _DEBUG )  cout << "SET the joint values!" << endl;

    return kinematic_state_ptr->satisfiesBounds();
}

bool FK_IK_Service::FK_cb( ur_motion_planning::FK::Request& req, ur_motion_planning::FK::Response& rsp ){

    bool valid  = load_q( req.req ) ,
         _DEBUG = true              ;

    if( _DEBUG )  cout << "FK service invoked!" << endl;

    if( valid ){
        const Eigen::Affine3d& end_effector_state = kinematic_state_ptr->getGlobalLinkTransform( end_link_name );

        // if( _DEBUG )  cout << "End Effector Pose:" << endl << end_effector_state << endl;

        Eigen::Matrix4d coords = Affine3d_to_homog( end_effector_state );

        if( _DEBUG ){
            cout << "Homogeneous:" << endl << coords << endl;
            std::cout << "IsRowMajor?: " << yesno( coords.IsRowMajor ) << std::endl;
            // std::cout << "IsColMajor?: " << yesno( coords.IsColMajor ) << std::endl;
        }  

        u_char k = 0;
        for( u_char i = 0 ; i < 4 ; i++ ){
            for( u_char j = 0 ; j < 4 ; j++ ){
                if( _DEBUG )  cout << "i = " << (u_short) i << ", j = " << (u_short) j << ", coords(i,j)=" << coords(i,j) << endl;
                rsp.rsp.pose[k] = coords(i,j);
                k++;
            }
        }

        if( _DEBUG )  cout << "FK packed a response: " << rsp.rsp.pose << endl;

        return true;
    }else{
        return false;
    }
}

FK_IK_Service::~FK_IK_Service(){
    if( joint_model_group_ptr ) delete joint_model_group_ptr;
    // if( kinematic_state_ptr )   delete kinematic_state_ptr;
}

void test_FK(){

}


/*************** MAIN ***************/

int main( int argc , char** argv ){
    // ROS set-ups:
    ros::init(argc, argv, "FK_IK", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type FK_IK_Service");
    FK_IK_Service serviceObj( nh );  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    
    // serviceObj.FKservice = nh.advertiseService( servName , &FK_IK_Service::FK_cb , &serviceObj );

    // Do our own spin loop
    while (!g_request_shutdown)
    {
        // Do non-callback stuff

        ros::spinOnce();
        usleep(1000);
    }

    // Do pre-shutdown tasks
    // delete &serviceObj;
    ros::shutdown();

    return 0;
} 