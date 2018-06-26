import cv2
from math import *
import numpy as np
import os
from os.path import join, basename
from scipy.linalg import block_diag
from collections import deque
import rospy
from geometry_msgs.msg import Twist
import ros
from multiprocessing import Pool
from geometry_msgs.msg import Pose2D


PI = 3.1415926535897
intrinsic_matrix=np.asarray([[700.881, 0.0, 0.0, 0.0], [0.0, 700.881, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
flag = 0
frame_count = 0 
k=0
j=float(0)

h_primer = 0
h_primel = 0
angle = 0
ref_angle = 0
rel_angle = 0
ref = 0

#PID parameters
p_error = 0;
i_error = 0;
d_error = 0;
cte = 25
frm = (None,None)
acc_error=0
acc_angle=0


def angle_transform(angle):
    global ref_angle
    global k
    global rel_angle
    k = (angle - ref_angle)
    rel_angle = rel_angle + k
    ref_angle = angle
        
    #print rel_angle
    #ref_angle = angle
    return k


def sum_angle(angle):
    global acc_angle
    acc_angle=acc_angle+angle
    return acc_angle

def sum_error(error):
    global acc_error
    acc_error=acc_error+error
    return acc_error


#LaneDriving
def rotate(lor,my_angle,org_angle=True):
    #Starts a new node
    
    rospy.init_node('robot_cleaner', anonymous=True)
    #now=rospy.get_rostime()
    #t=rospy.Time.from_sec(time.time())
    #now=t.to_sec()
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    position_publisher=rospy.Publisher('/turtle1/pose',Pose2D,queue_size=10)
    pos_msg=Pose2D()
    vel_msg = Twist()

    error=prev_error=acc_error=0
    # Receiveing the user's input
    #print("Let's rotate your robot")
    speed = 0
    angle = (my_angle)
    kp=10.895538986907214
    ki=10.85105543949617
    kd=-0.40877102516701286
    
    if lor:
        clockwise=False
    else:
        clockwise=True

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    #angular_speed=1000
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    pos_msg.x=0
    pos_msg.y=0
    pos_msg.theta= sum_angle(angle)
    acc_angle=sum_angle(angle)
    #print pos_msg.theta

    # Checking if our movement is CW or CCW
    if clockwise:
        
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    position_publisher.publish(pos_msg)
    if (org_angle):

        while(np.abs(current_angle-relative_angle)>0.0001):
            error=relative_angle-current_angle

            t1=rospy.Time.now().to_sec()
            angular_speed=kp*error+ki*sum_error(error)+kd*(error-prev_error)/0.08
            current_angle=angular_speed*0.08
            print relative_angle,current_angle
            #print (t1-t0)
            if clockwise:
                vel_msg.angular.z = -abs(angular_speed)
            else:
                vel_msg.angular.z = abs(angular_speed)
            velocity_publisher.publish(vel_msg)
            prev_error=error


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    pos_msg.x=0
    pos_msg.y=0
    #pos_msg.theta= current_angle
    #position_publisher.publish(pos_msg)
    #rospy.spin()


def PID_update(cte):
    Kp = 0.087;
    Ki = 0.001392;
    Kd = 1.359375;
    global  p_error 
    global  i_error 
    global  d_error 

    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    return -Kp*p_error - Ki*i_error #- Kd*d_error;


class LaneTracker:
    def __init__(self, n_lanes, proc_noise_scale, meas_noise_scale, process_cov_parallel=0, proc_noise_type='white'):
        self.n_lanes = n_lanes 
        self.meas_size = 4 * self.n_lanes
        self.state_size = self.meas_size * 2
        self.contr_size = 0

        self.kf = cv2.KalmanFilter(self.state_size, self.meas_size, self.contr_size)
        self.kf.transitionMatrix = np.eye(self.state_size, dtype=np.float32)
        self.kf.measurementMatrix = np.zeros((self.meas_size, self.state_size), np.float32)
        for i in range(self.meas_size):
            self.kf.measurementMatrix[i, i*2] = 1

        if proc_noise_type == 'white':
            block = np.matrix([[0.25, 0.5],
                               [0.5, 1.]], dtype=np.float32)
            self.kf.processNoiseCov = block_diag(*([block] * self.meas_size)) * proc_noise_scale
        if proc_noise_type == 'identity':
            self.kf.processNoiseCov = np.eye(self.state_size, dtype=np.float32) * proc_noise_scale
        for i in range(0, self.meas_size, 2):
            for j in range(1, self.n_lanes):
                self.kf.processNoiseCov[i, i+(j*8)] = process_cov_parallel
                self.kf.processNoiseCov[i+(j*8), i] = process_cov_parallel

        self.kf.measurementNoiseCov = np.eye(self.meas_size, dtype=np.float32) * meas_noise_scale

        self.kf.errorCovPre = np.eye(self.state_size)

        self.meas = np.zeros((self.meas_size, 1), np.float32)
        self.state = np.zeros((self.state_size, 1), np.float32)

        self.first_detected = False

    def _update_dt(self, dt):
        for i in range(0, self.state_size, 2):
            self.kf.transitionMatrix[i, i+1] = dt

    def _first_detect(self, lanes):
        for l, i in zip(lanes, range(0, self.state_size, 8)):
            self.state[i:i+8:2, 0] = l
        self.kf.statePost = self.state
        self.first_detected = True

    def update(self, lanes):
        if self.first_detected:
            for l, i in zip(lanes, range(0, self.meas_size, 4)):
                if l is not None:
                    self.meas[i:i+4, 0] = l
            self.kf.correct(self.meas)
        else:
            if lanes.count(None) == 0:
                self._first_detect(lanes)

    def predict(self, dt):
        if self.first_detected:
            self._update_dt(dt)
            state = self.kf.predict()
            lanes = []
            for i in range(0, len(state), 8):
                lanes.append((state[i], state[i+2], state[i+4], state[i+6]))
                
            return lanes
        else:
            return None



m = LaneTracker(2, 0.1, 500)


class sync():
    def __init__(self):
        self.ticks = 0
        self.prec_ticks = 0

    def process(self, lanes):
        self.prec_ticks = self.ticks
        self.ticks = cv2.getTickCount()
        dt = (self.ticks - self.prec_ticks) / cv2.getTickFrequency()

        predicted = m.predict(dt)
        
        #lanes = self.detect(frame)
        if lanes is not None:
            # print("Detected lanes in frame")
            m.update(lanes)

        if predicted is not None:
            return predicted[0], predicted[1]
        else:
            return None, None
        

    
s = sync()


def slope(x1, y1 , x2 , y2 ,h=1):
    #print y1-y2
    
    m =(x2-x1)**-1 * (y2-y1)
    if h==0:
      return m
    else:
      return np.arctan(m)*180*0.318


def Horizon(l,r,w,c=1):
    m = slope(l[0],l[1],l[2],l[3],0)
    n = slope(r[0],r[1],r[2],r[3],0)
    
    c = l[1]-l[0]*m
    d = r[1]-r[0]*n
    x = (-c+d)/(m-n)
    y = (d*m-c*n)/(m-n)
    #cv2.line(temp,(0,y),(w,y),(0,255,0),2)
    return x,y
   

def section(h,l,r):
    m = slope(l[0],l[1],l[2],l[3],0)
    n = slope(r[0],r[1],r[2],r[3],0)

    lfi =(h - l[1])*(m**-1)+l[0]
    rfi =(h - r[1])*(n**-1)+r[0]
    
    return lfi,rfi


def direction(m1,m2,o,p,temp):
    m1 = abs(m1)
    #TURN RIGHT
    #print abs(m1 - m2)
    if (m1 > m2 and m1 - m2 > 2):
        cv2.putText(temp, "Turn Right", ((o+p)/2-80,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255),2, lineType=cv2.LINE_AA) 
                
    elif(m2 > m1 and m2 -m1 > 2):
        cv2.putText(temp, "Turn Left", ((o+p)/2-80,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255),2, lineType=cv2.LINE_AA) 

    else:
        cv2.putText(temp, "Balanced", ((o+p)/2-80,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255),2, lineType=cv2.LINE_AA) 


def cross_track_error (h,l,k,y1,y2,p):

    global ref
    
    theta1 =  slope (h,l,y1,k,0)
    theta2 =  slope (h,l,y2,k,0)
    
    theta1 = theta1 * (180/PI)
    theta2 = theta2 * (180/PI)
    #print theta1,theta2
    rr = theta2
    ll = theta1
    ref_theta = abs(theta1)
    
    del_theta = (ref_theta - theta2)/2
    theta1 = del_theta + theta1
    theta2 = del_theta + theta2
    #print theta1,theta2

    
    theta1 = theta1 * (PI/180)
    theta2 = theta2 * (PI/180)

    y = k - l
    
    
    h1  = y2 + (theta2**-1)*(l-k)
    
    CTE = h1 - h

    req_angle = (np.arctan(CTE/y) * 180 * 0.318)
    
    
    angle  = req_angle - ref
    print angle


    """if angle < 0:
        rotate(False,np.abs(angle))

    elif angle > 0:
        rotate(True,np.abs(angle))"""


        
    """if(np.abs(ll)-rr<1):       
        
        if angle<0:
                rotate(False,np.abs(angle))
        elif angle>=0:
                rotate(True,np.abs(angle))
                
    elif(np.abs(ll)-rr>1):
        if angle<0:
                rotate(True,np.abs(angle))
        elif angle>=0:
                rotate(False,np.abs(angle))
    else :
        
        rotate(True,0)"""
            

 

    ref = req_angle 

    
def _process(img , temp):
    
    global cte
    global flag
    global frm
    global angle
    global frame_count 
    frame_count=frame_count+1

    left_slope = 0
    right_slope = 0

    angle_left = 0
    angle_right = 0
    
    x=0
    left_bound = None
    right_bound = None
    dum = img 
    #masking
    #ROI = img[img.shape[1]/2:img.shape[1],0:img.shape[0]]
    #cv2.imshow(ROI)


    
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50]) #example value
    upper_red = np.array([10,255,255]) #example value
    mask = cv2.inRange(img_hsv, lower_red, upper_red)
    img_result = cv2.bitwise_and(img, img, mask=mask)


    #thresholding
    
    
    dum = cv2.cvtColor(dum, cv2.COLOR_BGR2GRAY)
    sample = np.array([temp])
    pixel = np.average(sample)
    th1 = cv2.adaptiveThreshold(dum,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,15,2)
    th2 = cv2.adaptiveThreshold(dum,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,15,2)
    th = cv2.add(th1,th2)
    th = cv2.blur(img,(5,5))
    #print pixel 
    edges1 = cv2.Canny(th,50,150)
    
    #cv2.imshow("test",g)
    


    gray = cv2.cvtColor(img_result, cv2.COLOR_BGR2GRAY)
    kernel_size = 5

    #gauss_gray = cv2.GaussianBlur(kernel_size,(5,5),0)
    edges = edges1 #cv2.Canny(gray,50,150) 
    #edges2 = edges1
    #vertices = np.array([[(111,538),(491,276),(871,538)]])
    #edges, _ = region_of_interest(edges2, vertices)
    cv2.imshow("test",edges1)
    
    #Distance from the center calculator



    def distance(x1, y1, x2 , y2 , w , c):
        
        h = (img.shape[0])/2   

        if(x1!=x2):
          m = float(( (x2-x1)  * (y2-y1)**-1 ))
          x = x1 + (h-y1)*m
          y = h
          dist =  sqrt((x-w)*(x-w)+(y-h)*(y-h))
          
          
              
       
        elif(x1==x2 ):
          dist = (w-x1)
        elif(x1==x2 & c==1):
          dist = (x1-w) 
        return dist



    # Hough transform wrapper to return a list of points like PHough does
    lines = cv2.HoughLines(edges, 1, np.pi/180, 50)

    points = [[]]
    #if lines is None:
    #lines= cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=100, maxLineGap=10)
        
    if lines is not None:
        for l in lines:
            for rho, theta in l:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*a)
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*a)

                l = edges.shape[0] #Frame height



                if (y1!=y2):
                    if(x1!=x2):
                        m= ((x2-x1) * (y2-y1)**-1)
                        x= x1 + m*(edges.shape[0]-y1)
                    elif(x1==x2):
                        x = x1 
                        
                    if(x < edges.shape[1]*0.5):

                      if(theta > 0.3):
                         points[0].append((x1, y1, x2, y2))
                      
                         dist = distance(x1,y1,x2,y2,img.shape[1]*0.5,0)          
                      
                         if left_bound is None:
                           left_bound = (x1, y1, x2, y2)
                           left_dist = dist
                      
                         if left_bound is not None and dist < left_dist:
                           left_bound = (x1, y1, x2, y2)
                           left_dist = dist
                           
                    elif(x > edges.shape[1]*0.5):
                    
                      if(theta > 0.3):
                         points[0].append((x1, y1, x2, y2))
                      
                         dist = distance(x1,y1,x2,y2,img.shape[1]*0.5,1)          
                      
                         if right_bound is None:
                           right_bound = (x1, y1, x2, y2)
                           right_dist = dist
                      
                         if right_bound is not None and dist < right_dist:
                           right_bound = (x1, y1, x2, y2)
                           right_dist = dist         
                else:
                    right_bound  = None
                    left_bound = None
                  
            
            


            
    #lane = (left_bound,right_bound)
    x=0
    c=0
    if left_bound is not None:
        x = slope(left_bound[0],left_bound[1],left_bound[2],left_bound[3], edges.shape[1])
        
    if right_bound is not None:
        c = slope(right_bound[0],right_bound[1],right_bound[2],right_bound[3], edges.shape[1])
        



    lane = [left_bound,right_bound]
    
    if (-90 < x < -25 and 25 < c < 90):  #Slopes of the lines 
        lane= (left_bound,right_bound)
        flag = 1
        frm = lane
               
   
    elif(flag == 1):
        flag = 0
        lane = frm
        
    else:
      lane = (None,None)    
          
        
    lb,rb = s.process(lane)
   
    if lb and rb is not None:
      x = slope(lb[0],lb[1],lb[2],lb[3], edges.shape[1])
      c = slope(rb[0],rb[1],rb[2],rb[3], edges.shape[1])
      
      k,l=Horizon((lb[0]+100,lb[1]+temp.shape[0]-200,lb[2]+100,lb[3]+temp.shape[0]-200),(rb[0]+100,rb[1]+temp.shape[0]-200,rb[2]+100,rb[3]+temp.shape[0]-200),temp.shape[1])
      o,p=section(temp.shape[0],(lb[0]+100,lb[1]+temp.shape[0]-200,lb[2]+100,lb[3]+temp.shape[0]-200),(rb[0]+100,rb[1]+temp.shape[0]-200,rb[2]+100,rb[3]+temp.shape[0]-200))
      cv2.circle(temp, ((o+p)/2,temp.shape[0]-100),8, (20, 215, 20), -1)
      
      cv2.line(temp,(temp.shape[1]/2,temp.shape[0]-125),(temp.shape[1]/2,temp.shape[0]-75),(0,0,255),2)
      cv2.line(temp,((o+p)/2,temp.shape[0]-100),(temp.shape[1]/2,temp.shape[0]-100),(0,0,255),2)

      direction(x,c,o,p,temp)
      
      cross_track_error(k,l,temp.shape[0],o,p,(o+p)/2)
      distance = ((o+p)/2-temp.shape[1]/2)
      
   
      
   
    if(lb is not None):      
         
         cv2.line(temp,(k,l),(o,temp.shape[0]),(0,255,0),2)
         

    if(rb is not None):                 
         
         cv2.line(temp,(k,l),(p,temp.shape[0]),(0,255,0),2)
                  

    
    
    return temp

#ROSCVBRIDGE 

class ImageCvBridge(object):
    def __init__(self):
        self.cap = cv2.VideoCapture('seg4.mp4')
        #rospy.loginfo("Changing ROS topic to cv image...")
        #self.image_sub = rospy.Subscriber("/zed/rgb/image_rect_color",Image, self.image_callback)
        #self.bridge = CvBridge()
        #rospy.loginfo("all objects for ros to cv conversion initialised...")
        rospy.sleep(4)
        
        
        #cap_ = cv2.VideoCapture('seg6.mp4')

    def image_callback(self, data):
        self.image_data = data



    def run(self):
        ret, cv_image = self.cap.read() #self.bridge.imgmsg_to_cv2(self.image_data, "bgr8")
        cv_image = cv2.resize(cv_image,(660,356))
        frame = cv_image
        frame = frame[frame.shape[0]-200:frame.shape[0],100:frame.shape[1]-50]
        result = _process(frame,cv_image)       
        cv2.imshow('frame',result)
        if cv2.waitKey(10) & 0xFF == ord('q'):
          cv2.destroyAllWindows()
        

    def do_work(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.run()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('image_extracter')
    obj = ImageCvBridge()
    obj.do_work()

cv2.destroyAllWindows()
