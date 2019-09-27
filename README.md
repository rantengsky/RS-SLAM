# RS-SLAM
This is a semantic SLAM system that is robust in dyanmic environments.

1. Compiling the RS_SLAM in a workspace such as /catkin_ws, and get a RGBD node as well as semantic_cloud node.   
2. Putting the semantic_slam in ROS workspace, then use roslaunch to launch the semantic segmentation node.
Download the model in 
[model trained on ade20k](https://drive.google.com/file/d/1u_BEWdVIYiDnpVmAxwME1z3rnWWkjxm5/view?usp=sharing) /
[model trained on sunrgbd](https://drive.google.com/file/d/1t26t2VHNOzmjH-0lDTdYzXBACOV_4-eL/view?usp=sharing) /
and put them in models.
3. Runing the two ROS node to subscibe the image tpoic.
3. Running a .bag file in TUM3 database to publish rgb and depth images or the driver of openni if you have a RGB-D camera.
## Acknowledgement
This work cannot be done without many open source projets. Special thanks to
<br />[semantic_slam](https://github.com/floatlazer/semantic_slam)
<br />[ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
<br />[ORB_SLAM2_SSD_Semantic](https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic)
## License
This project is released under a GPLv3 license.
