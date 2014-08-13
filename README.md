dense_vision_client
===========================

Client for sending RGB+Depth images to aragorn for dense vision tracking and receiving back the tracked object poses.


### Launching with the Kinect ###


* Launch the kinect on the client computer (e.g. on the PR2 using `roslaunch /etc/ros/openni_head.launch`). 
 
* Edit the kinect launch file contained in this package (`./launch/kinect_dense_vision_client.launch`) file of this folder to make sure that the rgb and disparity topic remappings match those of your kinect camera:
 
        <remap from="/kinect_dense_vision_client/rgb" to="/camera/rgb/image_rect_color" />
        <remap from="/kinect_dense_vision_client/disparity" to="/camera/depth_registered/disparity" />


* Execute the **roslaunch** file contained in the `/launch` folder of this package:
    
        roslaunch dense_vision_client kinect_dense_vision_client.launch

