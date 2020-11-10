#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>

using std::string;
using std::cout;
using std::endl;

string get_file_string( string path ); // Get the contents of the file at `path` as a string

double randrange( double rMin = 0.0 , double rMax = 1.0 ); // Get a pseudo-random number between `rMin` and `rMax`

#endif