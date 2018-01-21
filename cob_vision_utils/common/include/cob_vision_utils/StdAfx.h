/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#ifdef __LINUX__
	// Void
#else
#pragma message("###############################################\n")
#pragma message("Compiling cob_vision_utils precompiled headers.\n")
#pragma message("###############################################\n")
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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/legacy/compat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/flann/hierarchical_clustering_index.h>


#include "cob_object_perception_intern/windows/src/extern/TinyXml/tinyxml.h"
#include "cob_object_perception_intern/windows/src/extern/TinyXml/tinystr.h"

#include <omp.h>
#endif
