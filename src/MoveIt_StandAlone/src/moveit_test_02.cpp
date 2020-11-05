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


int main(int argc, char** argv){

    string robot_desc = "/home/jwatson/libs/me530646labs/ur5/urdf/rviz/ur5.urdf";

    cout << "About to create model ..." << endl;
    urdf::Model model;

    cout << "About to load URDF ..." << endl;
    bool success = model.initFile( robot_desc );

    if( success )
        cout << "TEST SUCCEEDED!" << endl;
    else
        cout << "TEST FAILED!" << endl;
}