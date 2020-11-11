#include "helpers.h"

string get_file_string( string path ){
    // Get the contents of the file at `path` as a string
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

double randrange( double rMin , double rMax ){
    // Get a pseudo-random number between `rMin` and `rMax`
    double f = (double) rand() / RAND_MAX;
    return rMin + f * (rMax - rMin);
}

bool check_exist( string path , string check ){
    // Use `stat` to check if something exists
    struct stat buffer;   
    bool /*- */ result;
    stat( path.c_str() , &buffer );

    if( check == "f" || check == "file" || check == "F" || check == "FILE" )
        result = S_ISREG( buffer.st_mode );
    else if( check == "d" || check == "directory" || check == "D" || check == "DIRECTORY" )
        result =  S_ISDIR( buffer.st_mode );
    else
        result = false;

    return result;
}

string yesno( bool condition ){  return ( condition ? "YES" : "NO" );  }

Eigen::Matrix4d Affine3d_to_homog( const Eigen::Affine3d& xform ){

    // https://stackoverflow.com/a/15528871
    Eigen::Matrix3d R = xform.rotation();
    // Find your Rotation Matrix
    Eigen::Vector3d T = xform.translation();
    // Find your translation Vector
    Eigen::Matrix4d Trans; // Your Transformation Matrix
    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Trans.block<3,3>(0,0) = R;
    Trans.block<3,1>(0,3) = T;

    return Trans;
}

