dense_vision_client
===========================

Client for sending RGB+Depth images to aragorn for dense vision tracking and receiving back the tracked object poses.


### Launching the client ###

* Launch the kinect on the client computer (e.g. on the PR2 using `roslaunch /etc/ros/openni_head.launch`). 
 
* Edit the launch file contained in this package (`./launch/dense_vision_client.launch`) file of this folder to make sure that the rgb and disparity topic remappings match those of your kinect camera:
 
        <remap from="/dense_vision_client/rgb" to="/camera/rgb/image_rect_color" />
        <remap from="/dense_vision_client/disparity" to="/camera/depth_registered/disparity" />


* Execute the **roslaunch** file:
    
        roslaunch dense_vision_client dense_vision_client.launch

### PrimeSense camera ###

For running with a PrimeSense camera you will need to use modified versions of the `rgbd_launch` and `openni2_launch` packages in order to enable disparity processing.

To get it all to work you can just download and merge [this rosinstall file] [1] to your catkin workspace.

You also need to modify the projection matrix of your (depth) camera instrinsic calibration file. You can compare your **yaml** file [to this one] [2] (look at the -21.38 value on the projection matrix).

[1]: http://csc.kth.se/~fevb/downloads/dense_vision_primesense.rosinstall
[2]: http://csc.kth.se/~fevb/downloads/depth_PS1080_PrimeSense_with_disparity.yaml