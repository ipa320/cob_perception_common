^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_image_flip
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.20 (2024-02-19)
-------------------

0.6.19 (2022-07-29)
-------------------

0.6.18 (2021-05-10)
-------------------

0.6.17 (2020-10-14)
-------------------

0.6.16 (2020-10-10)
-------------------
* Merge pull request `#101 <https://github.com/ipa320/cob_perception_common/issues/101>`_ from fmessmer/test_noetic
  test noetic
* reduce dependencies
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

0.6.15 (2020-03-18)
-------------------
* Merge pull request `#99 <https://github.com/ipa320/cob_perception_common/issues/99>`_ from fmessmer/ci_updates
  [travis] ci updates
* catkin_lint fixes
* Merge pull request `#98 <https://github.com/ipa320/cob_perception_common/issues/98>`_ from fmessmer/fix/boost_signals
  removed unused Boost signals dependency
* removed unused Boost signals dependency
* Contributors: Felix Messmer, fmessmer

0.6.14 (2019-08-06)
-------------------

0.6.13 (2019-03-14)
-------------------

0.6.12 (2018-07-21)
-------------------
* fixed opencv include names (`#90 <https://github.com/ipa320/cob_perception_common/issues/90>`_)
  fixed opencv include names
* Contributors: Richard Bormann

0.6.11 (2018-01-07)
-------------------
* Merge remote-tracking branch 'origin/indigo_release_candidate' into indigo_dev
* Merge pull request `#87 <https://github.com/ipa320/cob_perception_common/issues/87>`_ from ipa-fxm/fix/depend_pluginlib
  add missing dependency
* add missing dependency
* Merge pull request `#86 <https://github.com/ipa320/cob_perception_common/issues/86>`_ from mikaelarguedas/patch-1
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* use license apache 2.0
* Contributors: Mikael Arguedas, Richard Bormann, ipa-fxm, ipa-uhr-mk

0.6.10 (2017-07-20)
-------------------
* removed unnecessary cmake_modules in CMakeLists
* Contributors: Richard Bormann

0.6.9 (2017-07-18)
------------------
* remove obsolete dependencies to cmake_modules
* manually fix changelog
* reduce console logs
* comment std out
* fixed a direction sign bug with sin() in gravity mode
* fixed upside down rotation bug in gravity mode
* 180deg flip for gravity mode
* fixed image flip rotation against gravity
* Contributors: Richard Bormann, ipa-cob4-1, ipa-fxm, msh

0.6.8 (2016-10-10)
------------------
* added publisher to image flip for providing the inverse inplane rotation to subscribers to the upright image
* Merge branch 'indigo_dev' of github.com:ipa320/cob_perception_common into indigo_dev
* Contributors: Richard Bormann

0.6.7 (2016-04-01)
------------------
* added the possibility to turn DisparityImages
* fix remaining issues from `#54 <https://github.com/ipa320/cob_perception_common/issues/54>`_
* removing dependency to message_generation because no messages are generated in this package
* conversion to package format 2
* removed an obsolete parameter
* added point cloud rotation, i.e. the rotation of the ordered indexing matrix of the point cloud data
* added wait for transform because of non-flipping bug
* added standard output on point cloud flip
* hopefully corrected dependencies to message generation
* Contributors: Florian Weisshardt, Richard Bormann, ipa-fmw

0.6.6 (2015-06-17)
------------------

0.6.5 (2015-06-17)
------------------
* unify version number
* changed version number
* updated changelog
* complete revision of cob_image_flip, simple configuration from yaml file, rotaation modi: fixed angle and automatic angle determination from tf with option to round rotation in 90 deg steps
* Merge branch 'indigo_dev' of github.com:ipa320/cob_perception_common into 320-indigo_dev
* cleaning up
* added a visualization program for cob_object_detection_msgs::DetectionArray messages, displays as a MarkerArray in Rviz
* catkinizing
* catkinizing
* Contributors: Florian Weisshardt, Richard Bormann, rmb-om

0.6.4 (2014-09-17)
------------------
* 0.5.12
* update changelog
* cleanup changelog
* 0.5.11
* update changelog
* Contributors: Florian Weisshardt

0.6.3 (2014-09-08)
------------------

0.6.2 (2014-09-01)
------------------
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
