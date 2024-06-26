# Scene Calibration

<a rel="license" href="http://creativecommons.org/licenses/by-nc-nd/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-nd/4.0/88x31.png" />

* [Description](#Description)
* [Prerequisites](#Prerequisites)
* [Installation](#Installation)
* [Usage](#Usage)

## <a name="Description"></a>1. Description

ROS packge to generate and calibrate a custom frame(s) and get plane measurements.
## <a name="Prerequisites"></a>2. Prerequisites

* [ROS Noetic](http://wiki.ros.org/ROS/Installation)
* [aruco_detect](https://github.com/carlosmccosta/fiducials)
* [usb_cam](http://wiki.ros.org/usb_cam)


## <a name="Installation"></a>3. Examples

Go to your ROS workspace environment and git clone the package
```
git clone XXX
```
Install dependencies.
```
rosdep install
```
Built it.
```
catkin build
```


## <a name="Usage"></a>4. Usage

For the usage cases bellow, it was used the Aruco Marker ID 17 with length of 0.07.

### Generate Frame

This mode uses a target frame (in this example the fiducial17 of Aruco) to be measured. Afterwards, the output is exported in a launch file format with the target frame name override.
1. Launch the generate_frame.launch
```
roslaunch scene_calibration generate_frame.launch
```

2. Put the aruco on the place where the frame should be.
3. Press Enter and wait for the measurements
4. Check the result in ```/output/output.launch```.

### Generate Multi Frames

This mode uses a target frame (in this example the fiducial17 of Aruco) to be measured several times. Afterwards, the output is exported in a launch file format with the target frame name override. In this file, the second and the third frames will be related to the first one.
1. Launch the multi_generation_frames.launch
```
roslaunch scene_calibration multi_generation_frames.launch
```

2. Put the aruco on the place where the first frame should be.
3. Press Enter and wait for the measurements
4. Put the aruco on the place where the second frame should be.
5. Press Enter and wait for the measurements
6. Put the aruco on the place where the third frame should be.
7. Press Enter and wait for the measurements
8. Check the result in ```/output/output.launch```. In this file, the second and the third frames will be related to the first one.


### Generate multi_planes
This mode uses a target frame (in this example the fiducial17 of Aruco) to be measured several times. The first measurement defined the master reference frame and the others the children planes. The output are exported in format of launch and yaml for master frames and planes respectively.

2. Launch the multi_planes_measurements.launch
```
roslaunch scene_calibration multi_planes_measurements.launch
```

2. Put the aruco on the place where the master frame should be.
3. Press Enter and wait for the measurements
4. Put the aruco on the place where the first plane should be.
5. Press Enter and wait for the measurements
6. Put the aruco on the place where the second plane should be.
7. Press Enter and wait for the measurements
8. Check the result in ```/output/output.launch``` and ```/output/output.yaml```. In this file, the planes will be related to the master frame.
9. Test the output by running:

   ```roslaunch scene_calibration plane_vis.launch```


-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
<br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-nd/4.0/">Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License</a>.
