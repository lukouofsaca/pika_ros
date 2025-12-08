#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example script showing how to publish images to the image_top topic.
This can be used to test the data capture system with OpenCV camera sources.
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    rospy.init_node('example_image_publisher', anonymous=True)
    
    # Create publisher for image_top topic
    image_pub = rospy.Publisher('/camera/color/image_top/image_raw', Image, queue_size=10)
    
    bridge = CvBridge()
    rate = rospy.Rate(30)  # 30 Hz
    
    # Open camera (use 0 for default camera)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        return
    
    rospy.loginfo("Publishing images to /camera/color/image_top/image_raw")
    rospy.loginfo("Press Ctrl+C to stop")
    
    while not rospy.is_shutdown():
        # Capture frame from camera
        ret, frame = cap.read()
        
        if not ret:
            rospy.logwarn("Failed to capture frame")
            continue
        
        try:
            # Convert OpenCV image to ROS Image message
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera_top"
            
            # Publish the image
            image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")
        
        rate.sleep()
    
    # Release camera
    cap.release()
    rospy.loginfo("Camera released")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
