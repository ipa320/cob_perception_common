#ifdef __LINUX__
	// Void
#else
#pragma message("#####################################\n")
#pragma message("Compiling BASIC precompiled headers.\n")
#pragma message("#####################################\n")
#include <iostream>
#include <iomanip>
#include <limits>
#include <list>
#include <map>
#include <string>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <set>
#include <windows.h>
#include <deque>
#define _USE_MATH_DEFINES
#include <cmath>
#include <set>
#include <assert.h>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/progress.hpp>
#include <boost/timer.hpp>
#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/mpl/size.hpp>

#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv/highgui.h>
#include <opencv2/flann/hierarchical_clustering_index.h>

#include "cob_object_perception_intern/windows/src/extern/TinyXml/tinyxml.h"
#include "cob_object_perception_intern/windows/src/extern/TinyXml/tinystr.h"

#include <omp.h>
#endif
