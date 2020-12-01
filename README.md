# stereo_calib_pipeline
stereo for vo
1.首先分别两个相机的内参

rosrun camera_calibration cameracalibrator.py --size row_point_nums x col_point_nums --square 0.03 image:=/usb_cam_node/image_raw camera:=/usb_cam_node

2.利用ros-stereo calib采集图像数据

rosrun camera_calibration cameracalibrator.py --approximate 0.05 --size 8x6 --square 0.108 right:=/my_stereo/right/image_raw left:=/my_stereo/left/image_raw right_camera:=/my_stereo/right left_camera:=/my_stereo/left

3.~/project/tools/calibration/stereoCalibration 生成extrinsics.yml, intrinsics.yml文件

4.按照https://blog.csdn.net/weixin_37918890/article/details/95626004填写orb-slam双目的配置文件

5.用orb-slam跑出双目的结果, 以及ins/lidar的轨迹

ORB-SLAM的代码:~/project/ORB-SLAM2

6.handeye

http://jira.allride-ai.cn:8080/browse/PV4-748

7.转换的代码
~/project/tools/poseTrans



sudo openvpn --config openvpn.ovpn