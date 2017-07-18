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
 

#ifndef __IPA_CAMERASENSORTOOLBOX_H__
#define __IPA_CAMERASENSORTOOLBOX_H__

#ifdef __LINUX__
	#include "cob_vision_utils/CameraSensorDefines.h"
	#include "cob_vision_utils/CameraSensorTypes.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/CameraSensorDefines.h"
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/CameraSensorTypes.h"
#endif

#include <opencv2/core/core.hpp>
#include <map>
#include <boost/shared_ptr.hpp>

namespace ipa_CameraSensors {

/// Define smart pointer type for toolbox
class CameraSensorToolbox;
typedef boost::shared_ptr<CameraSensorToolbox> CameraSensorToolboxPtr;

/// A toolbox for common color cameras.
///	Provides generic functions like image undistortion.
/// Holds essential matrices like distortion, intrinsic and extrinsic matrix.
/// For each camera in the system a separate camera toolbox should be initialized.
class __DLL_LIBCAMERASENSORS__ CameraSensorToolbox
{
	public: 
		CameraSensorToolbox();	///< Constructor.
		~CameraSensorToolbox();	///< Destructor.

		CameraSensorToolbox(const CameraSensorToolbox& cameraSensorToolbox); ///< Copy constructor

		/// Overwritten assignment operator
		CameraSensorToolbox& operator= (const CameraSensorToolbox& cameraSensorToolbox); 

		/// Release all allocated memory.
		/// @return Return code
		virtual unsigned long Release();

		/// Initialize the camera sensor toolbox.
		/// The matrices are read from the specified xml configuration file.
		/// @param directory The director where the configuration resides, with ending '/'.
		/// @param cameraType The camera type
		/// @param cameraIndex The camera index
		/// @param imageSize The Size of the image returned by the camera
		/// @return Return code
		virtual unsigned long Init(std::string directory, ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, const CvSize imageSize);

		/// Initialize the camera sensor toolbox.
		/// @param intrinsicMatrices Intrinsic parameters [fx 0 cx; 0 fy cy; 0 0 1]
		/// @param distortionParameters Distortion coefficients [k1, k2, p1=0, p2=0]
		/// @param extrinsicMatrices 3x4 matrix of the form (R|t), where R is a 3x3 rotation matrix
		/// and t a 3x1 translation vector.
		/// @param undistortMapsX The output array of x coordinates for the undistortion map
		/// @param undistortMapY The output array of Y coordinates for the undistortion map
		/// @param imageSize The Size of the image returned by the camera
		/// @return Return code
		virtual unsigned long Init(const std::map<std::string, cv::Mat>* intrinsicMatrices,
									const std::map<std::string, cv::Mat>* distortionParameters,
									const std::map<std::string, cv::Mat>* extrinsicMatrices,
									const std::map<std::string, cv::Mat>* undistortMapsX,
									const std::map<std::string, cv::Mat>* undistortMapY,
									const CvSize imageSize);

		/// Returns a matrix of the camera's extrinsic parameters.
		/// The extrinsic matrix is a 4x3 matrix of the format (R|t), where R desribes a 3x3 rotation matrix and
		/// t a 3x1 translation vector.
		/// @param cameraType The camera type
		/// @param cameraIndex The camera index
		/// @return The OpenCV matrix that refers to the extrinsic parameters of the camera.
		virtual cv::Mat GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		/// Sets the camera's extrinsic parameters.
		/// The extrinsic matrix is a 4x3 matrix of the format (R|T), where R desribes a 3x3 rotation matrix and
		/// T a 3x1 translation vector.
		/// @param t_cameraType The camera type
		/// @param cameraIndex The camera index
		/// @param _translation 3x1 translation vector.
		/// @param _rotation 3x3 rotation matrix.
		/// @return Return code.
		virtual unsigned long SetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, 
			const cv::Mat& _rotation, const cv::Mat& _translation);

		/// Sets the camera's extrinsic parameters.
		/// The extrinsic matrix is a 4x3 matrix of the format (R|T), where R desribes a 3x3 rotation matrix and
		/// T a 3x1 translation vector.
		/// @param key The key/identifier within the map of extrinsic matrices
		/// @param _translation 3x1 translation vector.
		/// @param _rotation 3x3 rotation matrix.
		/// @return Return code.
		virtual unsigned long SetExtrinsicParameters(std::string key, 
			const cv::Mat& _rotation, const cv::Mat& _translation);

		/// Returns a matrix of the camera's intrinsic parameters.
		/// @param cameraType The camera type, the parameters are optimized with
		/// @param cameraIndex Index of the specified camera, the parameters are optimized with
		/// @return The OpenCV matrix that should refer to the intrinsic parameters
		virtual cv::Mat GetIntrinsicMatrix(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		/// Returns the distortion coefficients.
		/// The matrix is given by [k1, k2, p1, p2] where k1, k2 are radial distortion coefficients
		/// and p1, p2 are tangential distortion coefficients.
		/// @param cameraType The camera type, the parameters are optimized with
		/// @param cameraIndex Index of the specified camera, the parameters are optimized with
		/// @return The OpenCV matrix that refers to the distortion parameters.
		virtual cv::Mat GetDistortionParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		/// Returns the distortion map for x components
		/// For each y pixel, the undistorted location is specified within the distortion map.
		/// @param cameraType The camera type, the parameters are optimized with
		/// @param cameraIndex Index of the specified camera, the parameters are optimized with
		/// @return The distortion map for y components
		virtual cv::Mat GetDistortionMapY(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		/// Returns the distortion map for x components
		/// For each x pixel, the undistorted location is specified within the distortion map.
		/// @param cameraType The camera type, the parameters are optimized with
		/// @param cameraIndex Index of the specified camera, the parameters are optimized with
		/// @return The distortion map for x components
		virtual cv::Mat GetDistortionMapX(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		/// Initializes the distortion parameters.
		/// The following equations apply: (x,y,z) = R*(X,Y,Z) + t and
		/// x' = x/z, y' = y/z and u = fx*x' + cx, v = fy*y' + cy. The model might be extended
		/// with distortion coefficients to replace x' and y' with 
		/// x'' = x'*(1 + k1*r^2 + k2*r^4) + 2*p1*x'*y' + p2(r^2+2*x'^2)
		/// y'' = y'*(1 + k1*r^2 + k2*r^4) + p1(r^2+2*y'^2) + 2*p2*x'*y'
		/// @param cameraType The camera type, the parameters are optimized with
		/// @param cameraIndex Index of the specified camera, the parameters are optimized with
		/// @param _intrinsicMatrix The cameras intrinsic matrix
		/// @param _distortion_coeffs radial and tangential distortion coefficient
		/// @return Return code.
		virtual unsigned long SetIntrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, 
			const cv::Mat& _intrinsicMatrix, const cv::Mat& _distortion_coeffs);

		/// Initializes the distortion parameters.
		/// The following equations apply: (x,y,z) = R*(X,Y,Z) + t and
		/// x' = x/z, y' = y/z and u = fx*x' + cx, v = fy*y' + cy. The model might be extended
		/// with distortion coefficients to replace x' and y' with 
		/// x'' = x'*(1 + k1*r^2 + k2*r^4) + 2*p1*x'*y' + p2(r^2+2*x'^2)
		/// y'' = y'*(1 + k1*r^2 + k2*r^4) + p1(r^2+2*y'^2) + 2*p2*x'*y'
		/// @param key The key/identifier within the map of extrinsic matrices
		/// @param _intrinsicMatrix The cameras intrinsic matrix
		/// @param _distortion_coeffs radial and tangential distortion coefficient
		/// @return Return code.
		virtual unsigned long SetIntrinsicParameters(std::string key, const cv::Mat& _intrinsicMatrix, const cv::Mat& _distortion_coeffs);

		/// Removes distortion from an image.
		/// It is necessary to set the distortion coefficients prior to calling this function.
		/// @param t_cameraType The camera type
		/// @param cameraIndex Index of the specified camera, the parameters are optimized with
		/// @param src The distorted image.
		/// @param dst The undistorted image.
		/// @return Return code.
		virtual unsigned long RemoveDistortion(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex,
			const cv::Mat& src, cv::Mat& dst);

		/// Returns image coordinates (u,v) from (x,y,z) coordinates. 
		/// (x,y,z) is expressed within the cameras coordinate system.
		/// @param t_cameraType The camera type
		/// @param cameraIndex Index of the specified camera, the parameters are optimized with
		/// @param u image coordinate u
		/// @param v image coordinate v
		/// @param x x-coordinates in mm relative to the camera's coodinate system.
		/// @param y y-coordinates in mm relative to the camera's coodinate system.
		/// @param z z-coordinates in mm relative to the camera's coodinate system.
		/// @return Return code
		virtual unsigned long ReprojectXYZ(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex,
			double x, double y, double z, int& u, int& v);

	private:
		
		/// Converts the camera type enumeration value to a string.
		/// @param cameraType The camera type as enumeration
		/// @param cameraTypeString The resulting string
		/// @return Retun code
		virtual unsigned long ConvertCameraTypeToString(ipa_CameraSensors::t_cameraType cameraType, std::string &cameraTypeString);

		/// Parses the XML configuration file, that holds the camera settings
		/// @param filename The file name and path of the configuration file
		/// @param cameraType The camera type i.e. CAM_AVTPIKE or CAM_IC
		/// @param cameraIndex The index of the camera within the configuration file
		///		   i.e. AvtPikeCam_0 or ICCam_1
		/// @return Return value
		virtual unsigned long LoadParameters(const char* filename, ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		bool m_Initialized; ///< True, when the camera has sucessfully been initialized.

		std::map<std::string, cv::Mat> m_intrinsicMatrices;	///< Intrinsic parameters [fx 0 cx; 0 fy cy; 0 0 1]
		std::map<std::string, cv::Mat> m_distortionCoeffs;	///< Distortion coefficients [k1, k2, p1=0, p2=0]
		std::map<std::string, cv::Mat> m_extrinsicMatrices; ///< a map of 3x4 matrix of the form (R|T),
									/// where R is a 3x3 rotation matrix and T is a 3x1 translation vector.

		std::map<std::string, cv::Mat> m_undistortMapsX;	///< The output array of x coordinates for the undistortion map
		std::map<std::string, cv::Mat> m_undistortMapsY;	///< The output array of Y coordinates for the undistortion map

		CvSize m_ImageSize; ///< The size of the image that is returned
};

/// Creates, intializes and returns a smart pointer object for the toolbox.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ CameraSensorToolboxPtr CreateCameraSensorToolbox();

} // end namespace
#endif // __IPA_CAMERASENSORTOOLBOX_H__
