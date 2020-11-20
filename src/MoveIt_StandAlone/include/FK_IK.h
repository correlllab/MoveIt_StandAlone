/*
~~~ DEV PLAN ~~~
[Y] Config file
    [Y] Make a relative ROS path
[Y] FK Service
    [Y] Test
[Y] IK Service
    [Y] Test
[Y] Test: Feed FK to IK
    [Y] Find out if I had to change the input pose to the robot basis somehow
[~] Confer with Davis to test ROS Bridge
    [Y] Check if Isaac ROS bridge can subscribe to services
    [ ] If not, then implement basic msg Subscribers/Publishers
    [ ] Create example file
*/

#ifndef FK_IK_H
#define FK_IK_H


/***** Included Libs *****/

/** Standard **/
#include <fstream>
using std::ifstream;
#include <exception>
#include <signal.h>

/** Parsing **/
#include <boost/date_time.hpp>
#include <tinyxml.h>
#include <jsoncpp/json/json.h>

/** Basic ROS **/
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>

/** FK/IK/MP **/
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

/** Messages **/
#include <std_msgs/Float32MultiArray.h>

#include <ur_motion_planning/FK_req.h>
#include <ur_motion_planning/FK_rsp.h>
#include <ur_motion_planning/FK.h>

#include <ur_motion_planning/IK_req.h>
#include <ur_motion_planning/IK_rsp.h>
#include <ur_motion_planning/IK.h>

/** Transforms **/
#include <Eigen/Geometry>

/** Local **/
#include "helpers.h"

/*************** Helper Functions ***************/

// KDL::JntArray result;
// KDL::Frame end_effector_pose;

KDL::Frame request_arr_to_KDL_frame( const boost::array<double,16>& pose ); // Translate a flattened pose to a KDL pose

boost::array<double,6> KDL_arr_to_response_arr( const KDL::JntArray& jntArr );

/*************** class FK_IK_Service ***************/

class FK_IK_Service{

/***** Public *****/ public:

ros::ServiceServer FKservice;
ros::ServiceServer IKservice;
ros::Subscriber    sub_FK_req;
ros::Publisher     pbl_FK_rsp;
ros::Subscriber    sub_IK_req;
ros::Publisher     pbl_IK_rsp;

FK_IK_Service( ros::NodeHandle& _nh );

bool /*-*/ load_q( const boost::array<double,6>& q ); // Load joint config from an FK request
KDL::Frame calc_FK( const boost::array<double,6>& jointConfig );
bool /*-*/ check_q(); // Check that the currently-set joint config lies within the limits

boost::array<double,7> calc_IK( const boost::array<double,22>& );

bool FK_cb( ur_motion_planning::FK::Request& req, ur_motion_planning::FK::Response& rsp );
bool IK_cb( ur_motion_planning::IK::Request& req, ur_motion_planning::IK::Response& rsp );

void FK_msg_cb( const std_msgs::Float32MultiArray& requestArr );
void IK_msg_cb( const std_msgs::Float32MultiArray& requestArr );

bool init_services();
bool init_pub_sub();

~FK_IK_Service();

/***** Protected *****/ protected:

/** ROS Node **/
ros::NodeHandle _nh; 

/** JSON Params **/
string FK_srv_topicName     ,
       IK_srv_topicName     ,
       FK_req_topicName     ,
       FK_rsp_topicName     ,
       IK_req_topicName     ,
       IK_rsp_topicName     ,
       URDF_full_path       ,
       SRDF_full_path       ,
       URDF_contents        ,
       base_link_name       , 
       end_link_name        , 
       _PKG_NAME        = "ur_motion_planning";

/** Flags **/
bool pathsOK = false;

/** Robot Description **/
KDL::Chain    chain; // ---- Kin chain
u_char /*--*/ N_joints; // # of joints in the kin chain
KDL::JntArray ll , // ------ lower joint limits
              ul , // ------ upper joint limits
              q  ; // ------ Current joint config

/** FK Solver **/
KDL::ChainFkSolverPos_recursive* fk_solver; //(chain); // Forward kin. solver

/** IK Solver and Params **/
TRAC_IK::TRAC_IK* tracik_solver = nullptr;
u_short /*----*/ N_IKsamples;
double /*-----*/ IK_timeout   , 
                 IK_epsilon   ,
                 IK_seed_fuzz ;

/** Internal Functions **/
bool load_JSON_config();
bool load_URDF_SRDF();
bool setup_kin_chain();
bool setup_FK();
bool setup_IK();



};

#endif