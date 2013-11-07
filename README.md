Build Procedure
===============

Download the source
`git clone https://github.com/ipa320/cob_perception_common.git`

Add your path to your `~/.bashrc`. For example, if you cloned the file into your home directory:
`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/cob_perception_common`

Then run CMake
`mkdir build && cd build && cmake ..`

In the directory where you clone the files run `rosmake`
