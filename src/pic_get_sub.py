#!/usr/bin/env python
'''pic_get ROS Node'''
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import cv2
import os
import argparse

parser=argparse.ArgumentParser(description="get RGB image ang depth image from topics")
parser.add_argument('--name',type=str,default="2021_d",help="the folder name you want save the image in")
args = parser.parse_args()


bridge=CvBridge()
num=0
color_path="/home/sgy/Window_Image/"+args.name+"/color/"
depth_path="/home/sgy/Window_Image/"+args.name+"/depth/"

def callback(data_color, data_depth):
    '''pic_get Callback Function'''
    global num
    num+=1

    if num%10!=0:
        return

    color_cv_image = bridge.imgmsg_to_cv2(data_color)[:,:,::-1]
    color_name=args.name+"_%03d"%(num/10)+".jpg"
    cv2.imwrite(color_path+color_name,color_cv_image)

    depth_cv_image = bridge.imgmsg_to_cv2(data_depth)
    depth_name=args.name+"_%03d"%(num/10)+".png"
    cv2.imwrite(depth_path+depth_name,depth_cv_image)
    #print(num/10)

def listener():
    '''pic_get Subscriber'''
    rospy.init_node('pic_get', anonymous=True)
    if not os.path.isdir(color_path):
        os.makedirs(color_path)
    if not os.path.isdir(depth_path):
        os.makedirs(depth_path)

    color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 1, 0.1, allow_headerless=False)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
