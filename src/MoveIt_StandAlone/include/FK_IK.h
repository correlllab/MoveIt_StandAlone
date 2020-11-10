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

#include "helpers.h"

#include <fstream>
using std::ifstream;

#include <boost/date_time.hpp>
#include <tinyxml.h>
#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <ur_motion_planning/FK_req.h>
#include <ur_motion_planning/FK_rsp.h>
#include <ur_motion_planning/FK.h>


void IK_cb();

class FK_IK_Service{
public:

ros::ServiceServer FKservice;
ros::ServiceServer IKservice;

FK_IK_Service( ros::NodeHandle* nodehandle );

void init_subscribers();
void init_services();

void FK_cb( ur_motion_planning::FK_req& request, ur_motion_planning::FK_rsp& response );

protected:

ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

string FK_req_topicName ,
       IK_req_topicName ,
       FK_rsp_topicName ,
       IK_rsp_topicName ,
       _PKG_NAME        = "ur_motion_planning";

void load_JSON_config();



};

#endif