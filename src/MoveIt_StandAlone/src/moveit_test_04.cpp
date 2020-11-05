#include <string>
#include <iostream>

using std::string;
using std::cout;
using std::endl;

//////////////////////////

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

////////////////////////////////

#include "ros/ros.h"

/////////////////////////////

#include <Eigen/Geometry>

////////////////////////////////


///////////////////////////////

#include <fstream>
#include <sstream>

string get_file_string( string path ){
    std::ifstream f( path );
    string rtnStr;
    bool _DEBUG = false;
    if( f ){
        std::stringstream buf;
        buf << f.rdbuf();
        f.close();
        rtnStr = buf.str();
    }else{
        cout << "File " << path << " could NOT be opened!" << endl;
    }
    int strLen = rtnStr.length();
    if( strLen == 0 )
        cout << "WARN: " << path << " yielded an EMPTY string!" << endl;
    else if( _DEBUG )
        cout << path << " yielded " << strLen << " characters.""" << endl;
    return rtnStr;
}

////////////////////////////


geometry_msgs::Pose ROS_pose_from_Eigen_xform( const Eigen::Affine3d& xform ){

    geometry_msgs::Pose rtnPose;

    Eigen::Vector3d trans = xform.translation();

    rtnPose.position.x = trans[0];
    rtnPose.position.y = trans[1];
    rtnPose.position.z = trans[2];

    Eigen::Quaterniond q( xform.rotation() );

    rtnPose.orientation.w = q.w();
    rtnPose.orientation.x = q.x();
    rtnPose.orientation.y = q.y();
    rtnPose.orientation.z = q.z();

    return rtnPose;
}


////////////////////////////
// common.gazebo.xacro                  ur10.urdf.xacro                     ur3.urdf.xacro                      ur5.urdf.xacro
// ur10_joint_limited_robot.urdf.xacro  ur3_joint_limited_robot.urdf.xacro  ur5_joint_limited_robot.urdf.xacro  ur.gazebo.xacro
// ur10_robot.urdf.xacro                ur3_robot.urdf.xacro                ur5_robot.urdf.xacro                ur.transmission.xacro



int main(int argc, char** argv){

    ros::init(argc, argv, "FK_IK");

    string URDF = get_file_string( "/media/jwatson/FILEPILE/Cpp/ws_MoveIt/src/MoveIt_StandAlone/resource/ur5.urdf" );
    string SRDF = get_file_string( "/media/jwatson/FILEPILE/Cpp/ws_MoveIt/src/MoveIt_StandAlone/resource/ur5.srdf" );

    robot_model_loader::RobotModelLoader::Options opt( URDF , SRDF );
    robot_model_loader::RobotModelLoader model( opt );

    robot_model::RobotModelPtr kinematic_model = model.getModel();

    cout << "Model frame: " << kinematic_model->getModelFrame().c_str() << endl;

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    
    const std::vector< std::string > jointModelNames = kinematic_model->getJointModelGroupNames();

    for( size_t i = 0 ; i < jointModelNames.size() ; i++ ){
        cout << "Joint Model " << i << ": " << jointModelNames[i] << endl;
        cout << "Has Group?: " << kinematic_model->hasJointModelGroup( jointModelNames[i] ) << endl << endl;
    }

    cout << "Check A" << endl;

    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    const std::vector<std::string> &link_names = kinematic_model->getLinkModelNames();

    cout << "Check B" << endl;

    for( size_t i = 0 ; i < joint_names.size() ; i++ ){
        cout << "Joint " << i << ": " << joint_names[i] << endl;
    }

    for( size_t i = 0 ; i < link_names.size() ; i++ ){
        cout << "Link " << i << ": " << link_names[i] << endl;
    }


    kinematic_state->setToRandomPositions( joint_model_group );

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    cout << "Check C" << endl;

    for(std::size_t i = 0; i < joint_names.size(); ++i){
        cout << "Check " << i << endl;
        printf("Joint %s: %f\n", joint_names[i].c_str(), joint_values[i]);
        // joint_values[i] = 0.5;
    }

    cout << "Is this a valid configuration?: " << kinematic_state->satisfiesBounds() << endl;

    // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("wrist_3_link");

    cout << endl << "=== FK ==="  << endl  << endl;

    /* Print end-effector pose. Remember that this is in the model frame */
    cout << "Translation: " << end_effector_state.translation() << endl;
    cout << "Rotation: " << end_effector_state.rotation() << endl;

    cout << endl << "=== IK ==="  << endl  << endl;


    kinematic_state->update();

    bool found_ik = false;
    if( 1 ){
        geometry_msgs::Pose pose = ROS_pose_from_Eigen_xform( end_effector_state );
        for( int i = 0 ; i < 100 ; i++ ){
            found_ik = kinematic_state->setFromIK( joint_model_group , pose , "wrist_3_link" , 500 , 10.0 );    
            cout << found_ik << ", " << std::flush;
        }
        cout << endl;
    }else{
        // found_ik = kinematic_state->setFromIK( joint_model_group , end_effector_state , 20, 0.5 );
    }

    if( found_ik ){
        printf("FOUND IK solution\n");
        // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        // for(std::size_t i=0; i < joint_names.size(); ++i)
        // {
        //     printf("Joint %s: %f\n", joint_names[i].c_str(), joint_values[i]);
        // }
    }else{
        printf("Did NOT find IK solution\n");
    }

}