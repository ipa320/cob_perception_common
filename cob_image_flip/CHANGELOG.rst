^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_image_flip
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update package.xml
* increase version number for indigo
* 0.5.10
* update changelog
* downgrade version for hydro
* using opencv2 instead of libopencv-dev for hydro version (should not be merged into indigo)
* Contributors: Florian Weisshardt

0.6.1 (2014-08-28)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* fix wrong opencv dep - again
* Contributors: Florian Weisshardt

0.6.0 (2014-08-28)
------------------
* 0.5.6
* 0.5.5
* update changelog
* missing install tags
* 0.5.5
* Merge pull request `#26 <https://github.com/ipa320/cob_perception_common/issues/26>`_ from ipa320/hydro_dev
  updates from hydro_dev
* update changelog
* fix wrong opencv dep
* Contributors: Alexander Bubeck, Florian Weisshardt, ipa-fxm

0.5.10 (2014-08-29)
-------------------
* downgrade version for hydro
* using opencv2 instead of libopencv-dev for hydro version (should not be merged into indigo)
* 0.6.1
* update changelog
* Merge branch 'indigo_dev' into indigo_release_candidate
* fix wrong opencv dep - again
* 0.6.0
* update changelog
* 0.5.5
* Merge pull request `#26 <https://github.com/ipa320/cob_perception_common/issues/26>`_ from ipa320/hydro_dev
  updates from hydro_dev
* update changelog
* fix wrong opencv dep
* Contributors: Alexander Bubeck, Florian Weisshardt

0.5.5 (2014-08-28)
------------------
* missing install tags
* Contributors: ipa-fxm

0.5.4 (2014-08-25)
------------------
* unify version number
* update changelog
* merge conflict, undo changes
* Missing dependencies and fixed error image_flip.launch
* merge from rmb
* added possibilities to manually command 90, 180, 270 deg image rotation
* merge with latest rmb changes
* merge with latest changes from rmb
* catkin adaptation
* pcl_conversionsupdated
* link PCL_LIBRARIES to targets
* include PCL as system dependency. Avoids build error on first catkin_make
* fixes for hydro
* fix include dirs
* catkinize cob_image_flip, not working due to dependency to cob_perception_utils
* convert stack to metapackge, ignore all pacakges within for now
* working on groovy transition
* added connect/disconnect callbacks
* add arg for nodelet manager
* uses arg instead of env variable for robot
* point cloud is not flipped by default now
* added boost signals link for nodelet version
* search for bugs
* link against boost::signals
  this is required to compile under fuerte
* parametrized display output
* added nodelet
* further timing statistics
* added timing measurements to image_flip
* added parameters, now provides pointcloud and image flip
* adding parameters to image_flip
* modifications for robot usage
* added an image flip component to image_flip
* updated cob_image_flip to flip PointCloud2 mit XYZRGB data type
* changed license
* fixed problems, function tested
* moved kinect image flip from cob_camera_sensors to cob_image_flip
* Contributors: Florian Weisshardt, Martin Günther, Richard Bormann, Srinivas Kerekare, ipa-fmw, ipa-goa, ipa-mig, ipa-nhg, rmb