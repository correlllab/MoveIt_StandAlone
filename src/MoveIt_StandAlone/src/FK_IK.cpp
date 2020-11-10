#include "FK_IK.h"



/*************** Class Method Definitions ***************/

FK_IK_Service::FK_IK_Service( ros::NodeHandle* nodehandle ){
    ROS_INFO("FK/IK Node starting ...");
}

void FK_IK_Service::load_JSON_config(){
    Json::Reader reader;
    Json::Value  obj;
    string /*-*/ pkgPath = ros::package::getPath( _PKG_NAME ) ;
    string /*-*/ configPath = pkgPath + "config.json";
    ifstream     ifs( configPath );
    
    reader.parse( ifs , obj ); // reader can also read strings

    FK_req_topicName = obj[ "UR_FKrequest_TOPIC"  ].asString();
    IK_req_topicName = obj[ "UR_IKrequest_TOPIC"  ].asString();
    FK_rsp_topicName = obj[ "UR_FKresponse_TOPIC" ].asString(); 
    IK_rsp_topicName = obj[ "UR_IKresponse_TOPIC" ].asString(); 
}

void FK_IK_Service::FK_cb( ur_motion_planning::FK_req& req, ur_motion_planning::FK_rsp& rsp ){

}

void FK_IK_Service::init_subscribers()
{
    ROS_INFO ("Initializing Subscribers ..." );
    // FKservice = nh_.advertiseService( FK_req_topicName , &FK_IK_Service::FK_cb, this);
    // req_FK = ros::Subscriber<ur_motion_planning::( nh_ , FK_req_topicName , 1 );
    // req_FK.registerCallback( FK_cb );  
    // add more subscribers here, as needed
}

/*************** MAIN ***************/

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "FK_IK"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type FK_IK_Service");
    FK_IK_Service serviceObj(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    string servName = "serv" ;
    serviceObj.FKservice = nh.advertiseService( servName , &FK_IK_Service::FK_cb , &serviceObj );

    ros::spin();
    return 0;
} 