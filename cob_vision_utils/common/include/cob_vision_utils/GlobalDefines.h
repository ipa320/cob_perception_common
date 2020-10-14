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
 

#ifndef __IPA_GLOBALDEFINES_H__
#define __IPA_GLOBALDEFINES_H__

namespace ipa_Utils {
	
/// An enum for the return values.
/// This enum describes possible return values that are used to return failure or success.
enum {
	RET_OK =									0x00000001UL, ///< Everythings OK.
	RET_FAILED =								0x00000002UL,  ///< Something went dramatically wrong.
	RET_WARNING =								0x00000004UL  ///< Something went wrong.
};

} // end namespace ipa_Utils

#endif // __IPA_GLOBALDEFINES_H__

