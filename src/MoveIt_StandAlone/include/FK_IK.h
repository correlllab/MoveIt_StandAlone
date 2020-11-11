/*
~~~ DEV PLAN ~~~
[~] Config file
    [ ] Make a relative ROS path
[ ] FK Service
    [ ] Test
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

/** Parsing **/
#include <boost/date_time.hpp>
#include <tinyxml.h>
#include <jsoncpp/json/json.h>

/** Basic ROS **/
#include <ros/ros.h>
#include <ros/package.h>

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

/** Transforms **/
#include <Eigen/Geometry>


#include "helpers.h"

void IK_cb();

/*************** class FK_IK_Service ***************/

class FK_IK_Service{

/***** Public *****/ public:

ros::ServiceServer FKservice;
ros::ServiceServer IKservice;

FK_IK_Service( ros::NodeHandle& _nh );

bool FK_cb( ur_motion_planning::FK::Request& req, ur_motion_planning::FK::Response& rsp );

bool init_services();

bool load_q( const ur_motion_planning::FK_req& q );

~FK_IK_Service();

/***** Protected *****/ protected:

ros::NodeHandle _nh; // we will need this, to pass between "main" and constructor

string FK_req_topicName     ,
       IK_req_topicName     ,
       FK_rsp_topicName     ,
       IK_rsp_topicName     ,
       URDF_full_path       ,
       SRDF_full_path       ,
       KDL_joint_group_name ,
       end_link_name        , 
       _PKG_NAME        = "ur_motion_planning";

bool   pathsOK = false;

u_char N_joints;

robot_model_loader::RobotModelLoader::Options opt;
robot_model_loader::RobotModelLoader /*----*/ robot_model;
robot_model::RobotModelPtr /*--------------*/ kinematic_model;
robot_state::JointModelGroup* /*-----------*/ joint_model_group_ptr;
robot_state::RobotStatePtr /*--------------*/ kinematic_state_ptr;

bool load_JSON_config();
bool setup_FK();



};

#endif