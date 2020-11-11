#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/stat.h>

/** Transforms **/
#include <Eigen/Geometry>

using std::string;
using std::cout;
using std::endl;

// typedef Eigen::MatrixXd<double,4,4,RowMajor> matx4d;

string get_file_string( string path ); // Get the contents of the file at `path` as a string

double randrange( double rMin = 0.0 , double rMax = 1.0 ); // Get a pseudo-random number between `rMin` and `rMax`

bool check_exist( string path , string check ); // Use `stat` to check if something exists

string yesno( bool condition ); // Return a string that describes the binary state

Eigen::Matrix4d Affine3d_to_homog( const Eigen::Affine3d& xform );

#endif