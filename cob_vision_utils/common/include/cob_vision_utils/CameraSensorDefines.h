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
 

// The following defines are used to determine which camera driver have been compiled 
// and are therefore available

#ifndef __IPA_CAMERASENSORDEFINES_H__
#define __IPA_CAMERASENSORDEFINES_H__

namespace ipa_CameraSensors {

#if defined _MSC_VER && _MSC_VER >= 1200
    // disable warnings related to inline functions
    #pragma warning( disable: 4251 4275)
#endif

/// Define, if we need to import or export the libraries
#ifdef __LINUX__
	#define __DLL_LIBCAMERASENSORS__ 
	#define APIENTRY
#else
	#ifdef __LIBCAMERASENSORS_EXPORT__
		#define __DLL_LIBCAMERASENSORS__ __declspec(dllexport)
	#else
		#define __DLL_LIBCAMERASENSORS__ __declspec(dllimport)
	#endif
#endif

} // namespace ipa_CameraSensors

#endif // __IPA_CAMERASENSORDEFINES_H__
