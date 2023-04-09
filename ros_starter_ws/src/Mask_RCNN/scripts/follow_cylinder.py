#! /usr/bin/env python

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist


# Coordinates For User/Target
y1 = 0;
x1 = 0;
y2 = 0;
x2 = 0;

def move_robot(vx, az):
    # Robot movement controller
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = az * 0.001
    # Publishing msg on rostopic /cmd_vel
    pub_twist.publish(twist)

def get_cylinder_distance(x1, x2, data):
    # Projecting camera data into lidar data to find distance between robot and cylinder inputs
    #  x1: Person detected bounding box first x coordinate
    #  x2: Person detected bounding box second x coordinate
    if x1 != 0 and x2 != 0:
        # lidar data has total 360 samples
        # theta1 and theta2 are the specific samples of the person
        theta1 = int(90 + 180/1020 * x1)
        theta2 = int(90 + 180/1020 * x2)
        # setting the threshold to 20: number of samples
        thresh = 20
        if theta1 > thresh and theta2 < (360-thresh):
            for idx in range(360-(theta2+thresh), 360-(theta1-thresh)):
                if data[idx] > 0.0 and data[idx] < 3.5:
                    return data[idx]
    return 0.0


def get_cylinder(input_image):
    # http://vanessavisionrobotica.blogspot.com/2018/02/
    # Mask the cylinder from raw rgb image
    global x1, x2, y1, y2
    # Gaussian filter (Soften the image)
    # Bluring src image as it comes with lot of noise
    gaussian_image = cv2.GaussianBlur(input_image, (5, 5), 0.2)

    # Color space conversion from RGB model to HSV
    # For different lighting condition
    # RGB values are highly sensitive to illumination making them not great for color detection.
    hsv_image = cv2.cvtColor(gaussian_image, cv2.COLOR_RGB2HSV)

    # Minimum and maximum values ​​of the green
    # Green: Our test/to-follow object to green
    value_min_HSV = np.array([50, 130, 130])
    value_max_HSV = np.array([80, 255, 255])

    # Filtering image
    # Applying the color filter with these minimum and maximum values ​​for hue, saturation, and intensity. 
    # Test/to-follow object -> green 
    # Rest objects in black.
    image_HSV_filtered = cv2.inRange(hsv_image, value_min_HSV, value_max_HSV)

    # Morphology is a broad set of image processing operations that process images based on shapes.
    # Close, morphology element
    # Improving threshold by filling unfiltered gaps
    kernel = np.ones((19, 19), np.uint8)
    image_HSV_filt_close = cv2.morphologyEx(image_HSV_filtered, cv2.MORPH_CLOSE, kernel)

    # Copying image
    image_HSV_filtered_Copy = np.copy(image_HSV_filt_close)
    input_image_Copy = np.copy(input_image)

    # Detection of object contour
    # Finding contours is like finding white object from black background.
    contours, _ = cv2.findContours(image_HSV_filtered_Copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    """
    We can use the command below to get the object 
    """    
    area = 0
    if (len(contours) != 0):
        cnt = cv2.approxPolyDP(contours[0], 3, True)
        rectX, rectY, rectW, rectH = cv2.boundingRect(cnt)

    for cnt in contours:
        # Approximates a polygonal curve(s) with the specified precision.
        cnt = cv2.approxPolyDP(cnt, 3, True)
        # Detect the rectangle that fits the detected boundary.  
        # It is a straight rectangle, it doesn't consider the rotation of the object. So area of the bounding rectangle won't be minimum.
        # Let (x,y) be the top-left coordinate of the rectangle and (w,h) be its width and height.
        x, y, w, h = cv2.boundingRect(cnt)
        if((x - w) * (y - h) > area):
            x1 = x
            y1 = y
            x2 = x + w
            y2 = y + h
        if (len(contours) != 0):
            cv2.rectangle(input_image_Copy, (x1, y1), (x2, y2), (255, 0, 0), 2)

    # Returning the masked image
    return input_image_Copy.astype(np.uint8)


def lidar_callback(msg):
    # Calculates cylinder distance using lidar and also moves the robot near to it
    global x1, x2
    dist = get_cylinder_distance(x1, x2, msg.ranges)
    w = float(1020.0/2.0 - (x1+x2)/2)
    # When it is within the range" Distance
    if dist > 1.0:
        print("Following Green Cylinder!")
        move_robot(0.05, w)
    elif dist > 0.0 and dist <= 1.0:
        print("Moving back!")
        move_robot(-0.05, w)
    elif dist > 0.0:
        move_robot(0.0, w)
    else:
        move_robot(0.0, 0.0)


def image_callback(img_msg):
    # Publishes resulting image
    # ROS image into numpy Array
    global model
    global vals
    global hal
    try:
        cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
    except:
        print("Skip!")
        return
    result_image = get_cylinder(cv_image)
    # Publish result image
    result_image_ros = img_msg
    result_image_ros.data = np.array(result_image, dtype=np.uint8).flatten().tolist()
    # Publishing the image
    pub_image.publish(result_image_ros)


# creating a ROS node
rospy.init_node('object_detection_node', anonymous=True)

# Subscriber: /rrbot/laser/scan
sub_lidar = rospy.Subscriber('/rrbot/laser/scan', LaserScan, lidar_callback)
# Subscriber: /camera/image_raw
sub_image = rospy.Subscriber("/camera/image_raw", Image, image_callback)
# Publisher: /camera/image_result
pub_image = rospy.Publisher("/camera/image_result", Image, queue_size=1)
# Publisher: /cmd_vel
pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


while not rospy.is_shutdown():
    rospy.spin()