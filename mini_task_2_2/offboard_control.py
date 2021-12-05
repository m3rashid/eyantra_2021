#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class offboard_control:
    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled"%e)
    
    def land_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to LAND
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. LAND Mode could not be set. Check that GPS is enabled"%e)
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def local_posCb(self, msg):
        # Callback function for topic /mavros/local_position/pose
        self.sp.position.x = msg.pose.position.x
        self.sp.position.y = msg.pose.position.y
        self.sp.position.z = msg.pose.position.z

def main():
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [] #List to setpoints
    pos0 = PoseStamped()
    pos1 = PoseStamped()
    pos2 = PoseStamped()
    pos3 = PoseStamped()
    pos4 = PoseStamped()

    pos0.pose.position.x = 0
    pos0.pose.position.y = 0
    pos0.pose.position.z = 10
    setpoints.append(pos0)

    pos1.pose.position.x = 10
    pos1.pose.position.y = 0
    pos1.pose.position.z = 10
    setpoints.append(pos1)

    pos2.pose.position.x = 10
    pos2.pose.position.y = 10
    pos2.pose.position.z = 10
    setpoints.append(pos2)

    pos3.pose.position.x = 0
    pos3.pose.position.y = 10
    pos3.pose.position.z = 10
    setpoints.append(pos3)

    pos4.pose.position.x = 0
    pos4.pose.position.y = 0
    pos4.pose.position.z = 10
    setpoints.append(pos4)

    # Create empty message containers 
    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here to 5m/s
    vel = Twist()
    vel.linear.x = 5
    vel.linear.y = 5
    vel.linear.z = 5
    
    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, stateMt.local_posCb)
    # Similarly initialize other subscribers 

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")

    # Publish the setpoints 
    for(i, sp) in enumerate(setpoints):
        local_vel_pub.publish(vel)
        local_pos_pub.publish(sp)
        rate.sleep()
        while(not rospy.is_shutdown()):
            if(abs(stateMt.sp.position.x - setpoints[i].pose.position.x) <= 0.1 and abs(stateMt.sp.position.y - setpoints[i].pose.position.y) <= 0.1 and abs(stateMt.sp.position.z - setpoints[i].pose.position.z) <= 0.1 and i != len(setpoints)-1):
                print("Reached setpoint", i)
                break

            elif(abs(stateMt.sp.position.x - setpoints[i].pose.position.x) <= 0.25 and abs(stateMt.sp.position.y - setpoints[i].pose.position.y) <= 0.25 and abs(stateMt.sp.position.z - setpoints[i].pose.position.z) <= 0.25 and i == len(setpoints)-1):
                print("Reached last setpoint")
                while not stateMt.state.mode=="AUTO.LAND":
                    ofb_ctl.land_set_mode()
                    rate.sleep()
                print ("LAND mode activated")
                break

            else:
                print("Reaching setpoint", i)
                rate.sleep()


    print("Mission completed")
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
