# RS-SLAM
**Authors:** T. Ran, L. Yuan, D. Tang 
This is a semantic SLAM system that is robust in dyanmic environments.
<img src="https://github.com/rantengsky/RS-SLAM/blob/master/pics/introduction-a.eps" width="375">
<img src="https://github.com/rantengsky/RS-SLAM/blob/master/pics/introduction-b.eps" width="375">
<img src="https://github.com/rantengsky/RS-SLAM/blob/master/pics/introduction-c.eps" width="375">
1. This project is built on ORB-SLAM2, so the Thirdparty and Vocabulary in ORB-SLAM2 should be copyed into rs-slam/. Then compiling the DBoW2 and g2o and uncompressing the ORBvoc.
2. Putting the whole project into the ROS workspace and running catkin_make to compile it. The RGBD node as well as a semantic_cloud node will be generated.   
2. Download the segmentation model in 
[model trained on sunrgbd](https://drive.google.com/file/d/1t26t2VHNOzmjH-0lDTdYzXBACOV_4-eL/view?usp=sharing)
and put them in semantic_slam/models/.
3. Runing the two ROS node to subscibe the image tpoic.
3. Running a .bag file in TUM3 database to publish rgb and depth images or the openni driver if you have a RGB-D camera.
## Dependencies
1. Pytorch 0.4.0 is required for semantic segmentation.
2. Ocotomap is required for map construction.
## Acknowledgement
This work cannot be done without many open source projets. Special thanks to
<br />[semantic_slam](https://github.com/floatlazer/semantic_slam)
<br />[ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
<br />[ORB_SLAM2_SSD_Semantic](https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic)
## License
This project is released under a GPLv3 license.
## Contact us
For any issues, please feel free to contact Teng Ran: rantengsky@163.com
