#include "FK_IK.h"

/*************** Class Method Definitions ***************/

FK_IK_Service::FK_IK_Service( ros::NodeHandle& _nh ){
    ROS_INFO("FK/IK Node starting ...");
    _nh = _nh;
    pathsOK = load_JSON_config();
    setup_FK();
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
        FK_req_topicName = obj[ "UR_FKrequest_TOPIC"  ].asString();
        IK_req_topicName = obj[ "UR_IKrequest_TOPIC"  ].asString();
        FK_rsp_topicName = obj[ "UR_FKresponse_TOPIC" ].asString(); 
        IK_rsp_topicName = obj[ "UR_IKresponse_TOPIC" ].asString(); 

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
        N_joints = joint_names.size();

        return true;

    }else{
        ROS_ERROR( "Could not start FK service due to incorrect paths. \nURDF: %s \nSRDF: %s" , URDF_full_path.c_str() , SRDF_full_path.c_str() );
    }
}

bool FK_IK_Service::init_services(){
    // string servName = "serv" ;
    FKservice = _nh.advertiseService( "serv" , &FK_IK_Service::FK_cb , this );
}

bool FK_IK_Service::load_q( const ur_motion_planning::FK_req& q ){
    std::vector<double> joint_values;
    for( u_char i = 0 ; i < N_joints ; i++ ){
       joint_values.push_back( q.q_joints[i] );
    }
    kinematic_state_ptr->setJointGroupPositions( joint_model_group_ptr , joint_values );
    return kinematic_state_ptr->satisfiesBounds();
}

bool FK_IK_Service::FK_cb( ur_motion_planning::FK::Request& req, ur_motion_planning::FK::Response& rsp ){

    bool valid = load_q( req.req );

    if( valid ){
        const Eigen::Affine3d& end_effector_state = kinematic_state_ptr->getGlobalLinkTransform( end_link_name );
        Eigen::Matrix4d coords = Affine3d_to_homog( end_effector_state );
        u_char k = 0;
        for( u_char i = 0 ; i < 4 ; i++ ){
            for( u_char j = 0 ; j < 4 ; j++ ){
                rsp.rsp.pose[k] = coords(i,j);
                k++;
            }
        }
        return true;
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
    ros::init(argc, argv, "FK_IK"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type FK_IK_Service");
    FK_IK_Service serviceObj( nh );  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    
    // serviceObj.FKservice = nh.advertiseService( servName , &FK_IK_Service::FK_cb , &serviceObj );

    ros::spin();
    return 0;
} 