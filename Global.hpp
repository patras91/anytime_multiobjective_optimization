#ifndef __GLOBAL__
#define __GLOBAL__

#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <cassert>
#include <climits>
#include <cfloat>
#include <cmath>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <list>
#include <utility>
#include <iomanip>
#include <limits>
#include <sstream>

#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>


#define PRINT_ERROR_LOC        cerr << "at " << __FILE__ << ":" << __LINE__\
                                    << " -> Function <" << __FUNCTION__\
                                    << ">" << endl;\
                               exit(0)

const size_t LEVEL 				   = 3;
const size_t VEC_DIMENSION         = LEVEL;            // size of the cost vector

//~ const size_t DEFAULT_X_DIM_MAX     = LEVEL;
//~ const size_t DEFAULT_Y_DIM_MAX     = LEVEL;
const size_t DEFAULT_DIM_MAX       = 27;			   // Maximum value of coordinate along one dimension
const size_t DEFAULT_NUM_DIM       = LEVEL;				
const size_t DEFAULT_NUM_RANGE     = LEVEL;

const size_t MAX_DIM               = LEVEL;           // Maximum number of dimensions

const size_t DBL_PRINT_WIDTH       = 8;
const size_t DBL_PRINT_PRECISION   = 2;

const size_t MAX_ARG_LEN           = 9;
const size_t TIME_OUT              = 900; 
const double DEFAULT_KAPPA_VAL     = 0.25;

const double MAX_DOUBLE_VAL        = 0.99;
const double MIN_DOUBLE_VAL        = 0.09; 

const size_t DEFAULT_DEPTH_BOUND   = 50000;

const double DEFAULT_L1_INC        = 10;
const double DEFAULT_SF_QUALITY    = 49;
const double INITIAL_SF_QUALITY    = 50;
const double FINAL_SF_QUALITY      = 100;
const double DEFAULT_SF_Q_INC      = 1;

//~ const size_t XDIM                  = 0;
//~ const size_t YDIM                  = 1;


const double EDGE_COST_MAX         = 10.0;
const double EDGE_COST_MIN         = 1.0;

const double m                     = 1;

using namespace std;

unsigned  GenNumInRange(unsigned max, unsigned min);
double    GenNumInRange(double max, double min);

#endif  //__GLOBAL__
