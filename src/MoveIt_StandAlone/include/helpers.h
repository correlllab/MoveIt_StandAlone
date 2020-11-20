#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/stat.h>
#include <boost/array.hpp>

/** Transforms **/
#include <Eigen/Geometry>

using std::string;
using std::cout;
using std::endl;
using std::ostream;
using std::vector;

// typedef Eigen::MatrixXd<double,4,4,RowMajor> matx4d;

string get_file_string( string path ); // Get the contents of the file at `path` as a string

double randrange( double rMin = 0.0 , double rMax = 1.0 ); // Get a pseudo-random number between `rMin` and `rMax`

bool check_exist( string path , string check ); // Use `stat` to check if something exists

string yesno( bool condition ); // Return a string that describes the binary state

Eigen::Matrix4d Affine3d_to_homog( const Eigen::Affine3d& xform );

template<typename T, std::size_t N> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const boost::array<T,N>& vec ){ // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    size_t len = vec.size();
    os << "[ ";
    for (size_t i = 0; i < len; i++) {
        os << vec[i];
        if (i + 1 < len) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const vector<T>& vec ){ // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    size_t len = vec.size();
    os << "[ ";
    for (size_t i = 0; i < len; i++) {
        os << vec[i];
        if (i + 1 < len) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

#endif