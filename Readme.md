# Build
In addition to ROS and PCL, you still need `libarchive` for dealing with the compressed velodyne data. `sudo apt install libarchive-dev`

`catkin_make`

Tested only on Ubuntu 20 and ROS Noetic.

# Usage
1. Download NCLT dataset using the official `downloader.py` (Attached an slightly modified version. Please modify `base_dir` on your favor). You can download ground truth, sensor data and velodyne only, as other information is not used by this program.
2. Create a directory named `rosbag` in the same directory, as doing so by hand is way more convenient than by C++ code.
3. use `./devel/lib/nclt2bag/nclt2bag PATH_TO_NCLT_DIRECTORY DATE` to convert the dataset. The result will be in the `rosbag` directory.

Note that `tf` is not published. You need to write your own tf publishing script.
The ground truth covariance is not used. Ground truth is aligned so that transformation of the first frame is identity.
