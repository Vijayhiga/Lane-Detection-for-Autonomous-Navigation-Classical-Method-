# Lane-Detection
This is the module for doing lane detection for DLive IIT Delhi. We have tested this on the zed_stereo_camera present on our car.

# Dependencies
[ROS CvBridge](http://wiki.ros.org/cv_bridge)

# Steps to run the file 
1) Make sure roscore is running

2) Run the ZED camera by the following command
  > roslaunch zed_wrapper zed.launch
   
3) Run the python file via terminal by the following command
  > python ZED3.py

For doing lane detection online, run the script **ZED3.py**. This subscribes to the **/zed/rgb/image_rect_color** topic so make sure your images are being published on this. If they are not you would have to change this topic in the script where the subscriber object is instantiated.

# Various concepts used
1) Canny Edge Filter    - Edge Detection with Kernal size of 3 
2) Hough Line Transform - Detection of Lanes 
3) Kalman Filter        - For real time tracking of the Lanes
4) PID Control          - For Steering Control of the car    

# Future Scope
Image segmentation using the **Convolutional Neutral Network - SegNet**, which can be used to detect lanes with very high accuracy compared to canny edge filter. 

# Results
DETECTION OF THE LANES  
![LANE](Results/frame.png "Lane detection")

EDGE DETECTION USING CANNY FILTER <br />
![CANNY](Results/imageedit_12_2827626451.png)

PID STEERING CONTROL (Visualized by Turtle sim bot) <br />
![PID](Results/Screenshot.png)



