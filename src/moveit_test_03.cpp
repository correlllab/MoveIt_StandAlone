#include <string>
#include <iostream>

using std::string;
using std::cout;
using std::endl;

//////////////////////////

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf/model.h>

////////////////////////////////


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


int main(int argc, char** argv){

    string URDF = get_file_string( "/home/jwatson/libs/universal_robot/ur_description/urdf/ur3.urdf.xacro" );
    string SRDF = get_file_string( "/home/jwatson/libs/universal_robot/ur5_moveit_config/config/ur5.srdf" );

    robot_model_loader::RobotModelLoader::Options opt( URDF , SRDF );

    robot_model_loader::RobotModelLoader model( opt );

    // string robot_desc = "/home/jwatson/libs/universal_robot/ur_description/urdf/ur3.urdf.xacro";

    // cout << "About to create model ..." << endl;
    // urdf::Model model;

    // cout << "About to load URDF ..." << endl;
    // bool success = model.initFile( robot_desc );

    // if( success )
    //     cout << "URDF EXISTS!" << endl;
    // else
    //     cout << "URDF NOT FOUND!" << endl;
}