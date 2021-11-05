#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

    Subsriptions					Publications
    /camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import aruco_library as library


class image_proc():
    def __init__(self): # Initialise everything
        rospy.init_node('marker_detection') #Initialise rosnode 
        self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1) # Making a publisher
        self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) # Subscribing to /camera/camera/image_raw
        self.img = np.empty([]) # This will contain your image frame from camera
        self.bridge = CvBridge()
        self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker

    # Callback function of amera topic
    def image_callback(self, data):
    # Note: Do not make this function lenghty, do all the processing outside this callback function
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image 
        except CvBridgeError as e:
            print(e)
            return
        
        (rows, cols, channels) = self.img.shape
        if cols > 60 and rows > 60:
            Detected_ArUco_markers = library.detect_ArUco(self.img) # Calling the function to detect ArUco marker
            ArUco_marker_angles = library.Calculate_orientation_in_degree(Detected_ArUco_markers) # Calling the function to calculate the orientation of the marker
            library.mark_ArUco(self.img, Detected_ArUco_markers, ArUco_marker_angles) # Calling the function to mark the ArUco marker

            key_list = Detected_ArUco_markers.keys()
            for key in key_list:
                dict_entry = Detected_ArUco_markers[key]
                centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
                centre[:] = [float(x / 4) for x in centre]
                centre = tuple(centre)
                self.marker_msg.id = int(key)
                self.marker_msg.x = centre[0]
                self.marker_msg.y = centre[1]
                self.marker_msg.z = 0.0
                self.marker_msg.roll = 0.0
                self.marker_msg.pitch = 0.0
                self.marker_msg.yaw = ArUco_marker_angles[key]
            
        cv2.imshow("Image window", self.img)
        cv2.waitKey(3)
        
            
    def publish_data(self):
        while not rospy.is_shutdown():
            try:
                rate = rospy.Rate(10) # 10hz
                self.marker_pub.publish(self.marker_msg)
                rate.sleep()
            except CvBridgeError as e:
                print(e)
                return

if __name__ == '__main__':
    image_proc_obj = image_proc()
    try:
        while not rospy.is_shutdown():
            image_proc_obj.publish_data()
            rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # image_proc_obj.publish_data()
    # cv2.destroyAllWindows()

