#include "FK_IK.h"

static bool _DEBUG = 1; // File-wide debug flag

/*************** Helper Functions ***************/

KDL::Frame request_arr_to_KDL_frame( const boost::array<double,16>& pose ){
    // Translate a flattened pose to a KDL pose
    size_t rotDex[9] = { 0, 1, 2,   4, 5, 6,  8, 9,10    }; // Rotation matrix
    size_t trnDex[3] = {          3,        7,        11 }; // Translation vector
    double rotTerms[9];
    double trnTerms[3];
    double rotTerm0 = pose[ rotDex[0] ];
    for( u_char i = 0 ; i < 9 ; i++ ){
        rotTerms[i] = pose[ rotDex[i] ];
        if( i < 3 )
            trnTerms[i] = pose[ trnDex[i] ];
    }
    return KDL::Frame( 
        KDL::Rotation( rotTerms[1] , rotTerms[1] , rotTerms[2] , rotTerms[3] , rotTerms[4] , rotTerms[5] , rotTerms[6] , rotTerms[7] , rotTerms[8] ) , 
        KDL::Vector( trnTerms[0] , trnTerms[1] , trnTerms[2] ) 
    );
}

const boost::array<double,16> KDL_frame_to_response_arr( KDL::Frame& pose ){
    // Translate a KDL pose to a flattened pose
    boost::array<double,16> rtnArr;
    u_char k = 0;
    for( int i = 0 ; i < 4 ; i++ ){
        for( int j = 0 ; j < 4 ; j++ ){
            rtnArr[k] = pose( i , j );
        }
    }
    return rtnArr;
}

boost::array<double,6> KDL_arr_to_response_arr( const KDL::JntArray& jntArr ){
    boost::array<double,6> rtnArr;
    for( size_t i = 0 ; i < 6 ; i++ ){  rtnArr[i] = jntArr(i);  }
    return rtnArr;
}

KDL::JntArray request_arr_to_KDL_arr( const boost::array<double,6>& jntArr ){
    KDL::JntArray rtnArr(6);
    for( size_t i = 0 ; i < 6 ; i++ ){  rtnArr(i) = jntArr[i];  }
    return rtnArr;
}

void fuzz_seed_array( KDL::JntArray& seedArr , double fuzz_rad ){
    size_t len = seedArr.rows();
    fuzz_rad = abs( fuzz_rad );
    for( size_t i = 0 ; i < len ; i++ ){
        seedArr(i) = seedArr(i) + randrange( fuzz_rad , fuzz_rad );
    }
}

/*************** Class Method Definitions ***************/

FK_IK_Service::FK_IK_Service( ros::NodeHandle& _nh ){
    ROS_INFO( "FK/IK Node starting ..." );
    _nh = _nh;
    pathsOK = load_JSON_config();
    load_URDF_SRDF();
    setup_kin_chain();
    setup_FK();
    setup_IK();
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

        // IK/FK
        base_link_name = obj[ "base_link" ].asString();
        end_link_name  = obj[ "end_link"  ].asString();
        N_IKsamples    = obj[ "N_IKsamples" ].asInt();
        IK_timeout     = obj[ "IK_timeout"   ].asDouble();
        IK_epsilon     = obj[ "IK_epsilon"   ].asDouble();
        IK_seed_fuzz   = obj[ "IK_seed_fuzz" ].asDouble();

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

bool FK_IK_Service::setup_kin_chain(){
    // Init the IK solver and retrieve the kin chain from it

    // 0. Init
    string contents   , // Processed contents of the URDF
           param_name , // Param search result (not used, but req'd arg)
           urdf_param = "/robot_description";
    bool   valid      = 0;

    // 2. Search for parameter and fetch its contents
    if( _nh.searchParam( urdf_param , param_name ) ){
        ROS_INFO("TEST: Found paramater");
        int i = 0;
        _nh.getParam( urdf_param , contents );
    }else{
        ROS_ERROR( ( "Could not find parameter!: " + urdf_param ).c_str() );
        return false;
    }  

    // 3. Init solver and retrieve the kinematic chain
    tracik_solver = new TRAC_IK::TRAC_IK( base_link_name , end_link_name , contents , IK_timeout , IK_epsilon );
    valid /*---*/ = tracik_solver->getKDLChain( chain );
    if( !valid ){  ROS_ERROR("There was no valid KDL chain found");  return false;  }

    // 4. Fetch the joint limits
    valid = tracik_solver->getKDLLimits( ll , ul );
    if( !valid ){  ROS_ERROR("There were no valid KDL joint limits found");  return false;  }

    // 5. Enforce that the joint limits are consistent with the number of joints
    N_joints = chain.getNrOfJoints();
    assert( N_joints == ll.data.size() );
    assert( N_joints == ul.data.size() );
    ROS_INFO( "SUCCESS: Loaded kinematic model with %d joints!" , N_joints );

    // 6. Init joints
    q = KDL::JntArray( N_joints );
    for( uint j = 0 ; j < N_joints ; j++ ){  q(j) = ( ll(j) + ul(j) ) / 2.0;  }

    return true;
}

bool FK_IK_Service::load_URDF_SRDF(){
    if( pathsOK ){
        
        // 1. Load *RDF
        string URDF = get_file_string( URDF_full_path );
        string SRDF = get_file_string( SRDF_full_path );

        // ROS complains if these are not set
        _nh.setParam( "/robot_description"          , URDF );
        _nh.setParam( "/robot_description_semantic" , SRDF );
        return true;

    }else{
        ROS_ERROR( "Could not start FK service due to incorrect paths. \nURDF: %s \nSRDF: %s" , URDF_full_path.c_str() , SRDF_full_path.c_str() );
        return false;
    }
}

bool FK_IK_Service::setup_FK(){
    // Init the FK solver
    fk_solver = new KDL::ChainFkSolverPos_recursive( chain ); // Forward kin. solver
    return 1;
}

bool FK_IK_Service::setup_IK(){
    /* NOTHING TO DO HERE */
    return 1;
}

bool FK_IK_Service::init_services(){
    // string servName = "serv" ;
    FKservice = _nh.advertiseService( FK_srv_topicName , &FK_IK_Service::FK_cb , this );
    IKservice = _nh.advertiseService( IK_srv_topicName , &FK_IK_Service::IK_cb , this );
}

bool FK_IK_Service::check_q(){
    // Check that the currently-set joint config lies within the limits
    for( u_char i ; i < N_joints ; i++ ){
        if(  ( q(i) < ll(i) )  ||  ( q(i) > ul(i) )  ) 
            return false;
    }
    return true;
}

bool FK_IK_Service::load_q( const ur_motion_planning::FK_req& Q ){

    double val = -50.0;

    if( _DEBUG ){
        cout << "Setting the joint model!" << endl;
        cout << "Got a vector with " << Q.q_joints.size() << " elements." << endl;
    }  

    for( u_char i = 0 ; i < N_joints ; i++ ){  q(i) = Q.q_joints[i];  }

    return check_q();
}

bool FK_IK_Service::FK_cb( ur_motion_planning::FK::Request& req, ur_motion_planning::FK::Response& rsp ){

    bool /*-*/ valid  = load_q( req.req );
    KDL::Frame end_effector_pose;

    if( _DEBUG )  cout << "FK service invoked!" << endl;

    if( valid ){

        fk_solver->JntToCart( q , end_effector_pose );

        rsp.rsp.pose = KDL_frame_to_response_arr( end_effector_pose );
        
        if( _DEBUG )  cout << "FK packed a response: " << rsp.rsp.pose << endl;

        return true;
    }else{
        ROS_ERROR( "Received an FK request OUTSIDE of loaded robot's joint limits!" );
        return false;
    }
}

bool FK_IK_Service::IK_cb( ur_motion_planning::IK::Request& req, ur_motion_planning::IK::Response& rsp ){

    if( _DEBUG )  cout << "Entered the IK callback!" << endl;

    KDL::Frame    reqFrame = request_arr_to_KDL_frame( req.req.pose );
    KDL::JntArray seedArr  = request_arr_to_KDL_arr( req.req.q_seed );
    KDL::JntArray result;

    if( _DEBUG )  cout << "Request vars loaded!" << endl;

    for( size_t i = 0 ; i < N_IKsamples ; i++ ){
        rsp.rsp.valid = tracik_solver->CartToJnt( seedArr , reqFrame , result );
        if( rsp.rsp.valid > 0 )
            break;
        else
            fuzz_seed_array( seedArr , IK_seed_fuzz );
    }

    if( _DEBUG )  cout << "Valid solution?: " << yesno( rsp.rsp.valid ) << ", ";

    rsp.rsp.q_joints = KDL_arr_to_response_arr( result );

    if( _DEBUG )  cout << "Solution Obtained: " << rsp.rsp.q_joints << endl;

    return ( rsp.rsp.valid > 0 );
}

FK_IK_Service::~FK_IK_Service(){
    if( tracik_solver ) delete tracik_solver;
    if( fk_solver )     delete fk_solver;
}


/*************** Node Setup ***************/

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig){  g_request_shutdown = 1;  }

// Replacement "shutdown" XMLRPC callback
void shutdownCallback( XmlRpc::XmlRpcValue& params , XmlRpc::XmlRpcValue& result ){
    int num_params = 0;
    if( params.getType() == XmlRpc::XmlRpcValue::TypeArray ){  num_params = params.size();  }
    if( num_params > 1 ){
        std::string reason = params[1];
        ROS_WARN( "Shutdown request received. Reason: [%s]" , reason.c_str() );
        g_request_shutdown = 1; // Set flag
    }
    result = ros::xmlrpc::responseInt( 1 , "" , 0 );
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

    

    // Do our own spin loop
    while (!g_request_shutdown){
        // Do non-callback stuff

        ros::spinOnce();
        usleep( 1000 );
    }

    // Do pre-shutdown tasks
    ros::shutdown();

    return 0;
} 