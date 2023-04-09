#! /usr/bin/env python
 
import cv2
import rospy
import numpy as np
import tensorflow as tf
graph = tf.get_default_graph()
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
 
import coco
import model as modellib
 
# Setting up the paths 
CLASS_NAMES = ['BG','person']
MODEL_DIR = "logs"
COCO_MODEL_PATH = "mask_rcnn_coco.h5"
 
# Coordinates For User/Target
y1 = 0;
x1 = 0;
y2 = 0;
x2 = 0;

N = 90
get_hist = True
vals = None
persondetected = False
img_cnt = 0
 
def move_robot(vx, az):
    # Robot movement controller
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = az * 0.001
    # print(twist.linear.x, twist.angular.z)
    # Publishing msg on rostopic /cmd_vel
    pub_twist.publish(twist)

 
def get_person_distance(x1, x2, data):
    # Projecting camera data into lidar data to find distance between robot and cylinder inputs
    #  x1: Person detected bounding box first x coordinate
    #  x2: Person detected bounding box second x coordinate
    if x1 != 0 and x2 != 0:
        # lidar data has total 360 samples
        # theta1 and theta2 are the specific samples of the person
        theta1 = int(90 + 180/1020 * x1)
        theta2 = int(90 + 180/1020 * x2)
        # setting the threshold to 20
        thresh = 20
        if theta1 > thresh and theta2 < (360-thresh):
            for idx in range(360-(theta2+thresh), 360-(theta1-thresh)):
                if data[idx] > 0.0 and data[idx] < 3.5:
                    return data[idx]
    return 0.0
 
def draw_instances(image, boxes, masks, class_ids, class_names, scores=None):
    # https://github.com/matterport/Mask_RCNN/pull/38
    # Masking the person from Raw RGB image
    global get_hist
    global vals
    global y1, x1, y2, x2
    
    global persondetected
 
    # Number of instances
    N = boxes.shape[0]
    
    if not N:
        persondetected = False
        print("Person not detected\n")
        return image
    else:
        assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]
    
    masked_image = image.copy()

    vals_arr = np.array([])

    for i in range(N):
        class_id = class_ids[i]
        score = scores[i] if scores is not None else None
        
        if not np.any(boxes[i]):
            continue
 
        try:
            label = class_names[class_id]
        except:
            continue
        # Class label check for the masked image    
        if label == "person" and score > 0.7:
            persondetected = True
        else:
            persondetected = False
            continue

        if not get_hist:
            try:
                # get hist of current person
                vals_arr = np.append(vals_arr, np.mean(vals - (masked_image).mean(axis=2).flatten()))            
            except:
                continue
        
        mask = masks[:, :, i]
        masked_image[mask==0.0]=0.0
    
    if not get_hist:
        try:
        # get bounding box coordinate with lowest difference in histogram with the original
            y1, x1, y2, x2 = boxes[np.argmin(vals_arr)]  
            cv2.rectangle(masked_image, (x1, y1),(x2, y2), (255, 0, 0), 2)
        except:
            pass

    if get_hist:
        # original hist of person
        vals = (masked_image).mean(axis=2).flatten()
        get_hist = False
        print("Received Histogram!")
    
    return masked_image.astype(np.uint8)
 
 
def lidar_callback(msg):
    # Calculates person distance using lidar and also moves the robot near to it
    global x1, x2
    global persondetected

    if not persondetected: 
        move_robot(0.0, 0.0)
        return
 
    dist = get_person_distance(x1, x2, msg.ranges)
    w = float(1020.0/2.0 - (x1+x2)/2)

    # When it is within the range
    if dist > 2.0:
        print("Following person!")
        move_robot(0.05, w)
    elif dist > 0.0 and dist <= 1.0:
        print("Moving back!")
        move_robot(-0.05, w)
    elif dist > 0.0:
        move_robot(0.0, w)
    else:
        move_robot(0.0, 0.0)
 
 
def image_callback(img_msg):
    # Detecting the person using the Mask-RCNN Pretained Model
    global model
    global vals
    global img_cnt

    img_cnt = img_cnt + 1
    if img_cnt % 30 != 0:
        return

    try:
        cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
    except:
        print("Skip!")
        return
    with graph.as_default():
        results = model.detect([cv_image], verbose=0)
    r = results[0]
    result_image = draw_instances(cv_image, r['rois'], r['masks'], r['class_ids'], CLASS_NAMES, r['scores'])
    # Publishing the resulting image
    result_image_ros = img_msg
    result_image_ros.data = np.array(result_image, dtype=np.uint8).flatten().tolist()
    pub_image.publish(result_image_ros)
 
 
# Class Required by Mask-RCNN Pretained Model
class InferenceConfig(coco.CocoConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
 
# Mask-RCNN Configuration
config = InferenceConfig()
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
model.load_weights(COCO_MODEL_PATH, by_name=True)
 
rospy.init_node('object_detection_node', anonymous=True)
 
# Subscriber: /rrbot/laser/scan
sub_lidar = rospy.Subscriber('/rrbot/laser/scan', LaserScan, lidar_callback)
# Subscriber: /camera/image_raw
sub_image = rospy.Subscriber("/camera/image_raw", Image, image_callback)
# Publisher: /camera/image_result
pub_image = rospy.Publisher("/camera/image_result", Image, queue_size=1)
# Publisher: /cmd_vel
pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# loop_rate = rospy.Rate(30)
while not rospy.is_shutdown():
    rospy.spin()
    # loop_rate.sleep()
 
 
# SAVES HISTOGRAM
from collections import Counter
# Filter the person from the background
vals = vals[vals != 0.0]
# Remove the value with most occurance for better plot
c = Counter(vals)
vals = vals[vals != c.most_common(1)[0][0]]
b, bins, patches = plt.hist(vals, 255)
plt.xlim([1,255])
plt.savefig("user_histogram.png")
print("Histogram saved!")