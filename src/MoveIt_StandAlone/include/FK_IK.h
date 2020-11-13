/*
~~~ DEV PLAN ~~~
[Y] Config file
    [Y] Make a relative ROS path
[Y] FK Service
    [Y] Test
[ ] IK Service
    [ ] Test
[ ] Test: Feed FK to IK
    [ ] Find out if I had to change the input pose to the robot basis somehow
[ ] Confer with Davis to test ROS Bridge
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

FK_IK_Service( ros::NodeHandle& _nh );

bool FK_cb( ur_motion_planning::FK::Request& req, ur_motion_planning::FK::Response& rsp );
bool IK_cb( ur_motion_planning::IK::Request& req, ur_motion_planning::IK::Response& rsp );

bool init_services();

bool load_q( const ur_motion_planning::FK_req& q );

~FK_IK_Service();

/***** Protected *****/ protected:

ros::NodeHandle _nh; // we will need this, to pass between "main" and constructor

string FK_srv_topicName     ,
       IK_srv_topicName     ,
       URDF_full_path       ,
       SRDF_full_path       ,
       URDF_contents        ,
       KDL_joint_group_name ,
       base_link_name       , 
       end_link_name        , 
       _PKG_NAME        = "ur_motion_planning";

bool   pathsOK = false;

u_char N_joints;

robot_model_loader::RobotModelLoader::Options opt;
robot_model_loader::RobotModelLoader /*----*/ robot_model;
robot_model::RobotModelPtr /*--------------*/ kinematic_model;
robot_state::JointModelGroup* /*-----------*/ joint_model_group_ptr = nullptr;
robot_state::RobotStatePtr /*--------------*/ kinematic_state_ptr;

TRAC_IK::TRAC_IK* tracik_solver = nullptr;
u_short /*----*/ N_IKsamples;
double /*-----*/ IK_timeout   , 
                 IK_epsilon   ,
                 IK_seed_fuzz ;

bool load_JSON_config();
bool setup_FK();
bool setup_IK();



};

#endif