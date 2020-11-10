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