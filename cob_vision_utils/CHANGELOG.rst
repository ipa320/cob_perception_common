^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_vision_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.8 (2016-10-10)
------------------

0.6.7 (2016-04-01)
------------------
* work on uEye and N30 integration
* added Ensenso N30 sensor
* fix remaining issues from `#54 <https://github.com/ipa320/cob_perception_common/issues/54>`_
* conversion to package format 2
* hopefully corrected dependencies to message generation
* Contributors: Richard Bormann, ipa-fmw

0.6.6 (2015-06-17)
------------------

0.6.5 (2015-06-17)
------------------
* Merge branch 'indigo_dev' of github.com:ipa320/cob_perception_common into 320-indigo_dev
* Merge pull request `#43 <https://github.com/ipa320/cob_perception_common/issues/43>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Adapted precompiled header settings
* catkinizing
* Adapted precompiled header filepath to ROS specs
* Using standard random number generator, replaced precompiled headers with references to a file within this metapackage. Adapted package description.
* Adapting optimizatino settings
* Cleaning up header files
* Added 16U as possible matrix type for image visualization
* Adapted project filestructure
* JSF: Adapted Kinect/Asus Xtion integration
* Contributors: Florian Weisshardt, Jan Fischer, Richard Bormann, ipa-jsf

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
* add tinyxml to package.xml
* Contributors: Florian Weisshardt

0.6.2 (2014-09-01)
------------------
* add tinyxml to package.xml
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
* merge with hydro
* missing install tags
* catkin_lint'ing
* 0.5.5
* Merge pull request `#26 <https://github.com/ipa320/cob_perception_common/issues/26>`_ from ipa320/hydro_dev
  updates from hydro_dev
* update changelog
* Merge branch 'hydro_dev' of github.com:ipa320/cob_perception_common into indigo_dev
* fix wrong opencv dep
* added install tags
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
* merge with hydro
* catkin_lint'ing
* 0.5.5
* Merge pull request `#26 <https://github.com/ipa320/cob_perception_common/issues/26>`_ from ipa320/hydro_dev
  updates from hydro_dev
* update changelog
* Merge branch 'hydro_dev' of github.com:ipa320/cob_perception_common into indigo_dev
* fix wrong opencv dep
* added install tags
* Contributors: Alexander Bubeck, Florian Weisshardt, ipa-fxm

0.5.5 (2014-08-28)
------------------
* missing install tags
* Contributors: ipa-fxm

0.5.4 (2014-08-25)
------------------
* unify version number
* add version number
* update changelog
* merge conflict, undo changes
* Missing dependencies and fixed error image_flip.launch
* merge from rmb
* fixes for hydro
* added OpenCV dependency
* remove leftover manifest.xml in cob_vision_utils
* fix include dirs
* catkinize cob_vision_utils
* convert stack to metapackge, ignore all pacakges within for now
* changed thickness of bounding box marker
* added detection_msg to marker_msg conversion
* Tabifying of file
* Merge of cob_vision_utils
* moved cob_vision_utils to cob_perception_common
* Contributors: Florian Weisshardt, Jan Fischer, Richard Bormann, ipa-goa, ipa-goa-sf, ipa-mig, ipa-nhg
