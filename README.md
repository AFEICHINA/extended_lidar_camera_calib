# expend_lidar_camera_calib

This work is an expend version of [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib.git), which is suitable for spinning LiDAR。

In order to apply this algorithm on spinning LIDAR(e.g:VLP16)， I adding the preprocess process([FLOAM](https://github.com/wh200720041/floam.git)) to make the point cloud of the spinning LiDAR denser.

## Build
```
cd ~/catkin_ws/src
git clone https://github.com/AFEICHINA/expend_lidar_camera_calib.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Run
step1: doing slam to accumulate dense pointcloud. 
```
roslaunch floam floam.launch
```


step2: lidar camera calibration
```
roslaunch livox_camera_calib calib.launch
```

## Acknowledgements
Thanks for [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib.git) and [FLOAM](https://github.com/wh200720041/floam.git).
