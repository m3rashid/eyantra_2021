
# Roadmap of the misson
# [ 0, 0, 0 ],  Takeoff
# [ 0, 0, 3 ], 
# [ 3, 0, 3 ],
# [ 3, 0, 0 ] Land on the box and pick it
# [ 3, 0, 3 ],
# [ 3, 3, 3 ], 
# [ 3, 3, 0 ],  Leave the box here and takeoff
# [ 3, 3, 3 ], 
# [ 0, 0, 3 ],
# Land at the position [ 0, 0, 0 ],


# /gripper_check      to check that the drone is above the box in the allowable range to pick the box

# If the result of gripper_check is true, to pick the box, you need to call the rosservice /activate_gripper and pass the value true to attach the box. If the box is attached, you will get a result from the rosservice as true. If the result is false, which means the drone is not correctly placed over the box

# To detach the box, you need to use the same rosservice /activate_gripper and pass the value false. This will detach the box from the drone

# to run       roslaunch task_3 task3_1.launch


import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# The gazebo world consist of a drone and a strawberry box.

# You have to put the drone in Offboard mode and publish the positions of the drone as setpoints.

# The location of the box is 3m 0m 0m, the drone needs to do the following

# Takeoff and the initial position to the height of 3m
# Go to the coordinate 3m 0m 3m
# Land on the box and pick the box
# Takeoff at the height of 3m and go to 3m 3m 3m
# Land at 3m 3m 0m and drop the box
# Takeoff again to height of 3m
# Head back to the start position ie 0m 0m 3m
# Finally land the drone at 0m 0m 0m and then disarm
# After landing on the box, you need to check that the drone is above the box in the allowable range to pick the box. To do that, you need to subscribe to the rostopic /gripper_check. If the value is true, you can now pick the box and if the value is false,the drone is not correctly placed and you need to correct the position.

# If the result of gripper_check is true, to pick the box, you need to call the rosservice /activate_gripper and pass the value true to attach the box. If the box is attached, you will get a result from the rosservice as true. If the result is false, which means the drone is not correctly placed over the box.

# To detach the box, you need to use the same rosservice /activate_gripper and pass the value false. This will detach the box from the drone

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
	pos5 = PoseStamped()
	pos6 = PoseStamped()
	pos7 = PoseStamped()

	pos0.pose.position.x = 0
	pos0.pose.position.y = 0
	pos0.pose.position.z = 3
	setpoints.append(pos0)

	pos1.pose.position.x = 3
	pos1.pose.position.y = 0
	pos1.pose.position.z = 3
	setpoints.append(pos1)

	pos2.pose.position.x = 3
	pos2.pose.position.y = 0
	pos2.pose.position.z = 0  		# Land on the box here and pick it up
	setpoints.append(pos2)

	pos3.pose.position.x = 3
	pos3.pose.position.y = 0
	pos3.pose.position.z = 3
	setpoints.append(pos3)

	pos4.pose.position.x = 3
	pos4.pose.position.y = 3
	pos4.pose.position.z = 3
	setpoints.append(pos4)

	pos5.pose.position.x = 3
	pos5.pose.position.y = 3
	pos5.pose.position.z = 0			# Land here and detach the box and takeoff
	setpoints.append(pos5)

	pos6.pose.position.x = 3
	pos6.pose.position.y = 3
	pos6.pose.position.z = 3
	setpoints.append(pos6)

	pos7.pose.position.x = 0
	pos7.pose.position.y = 0
	pos7.pose.position.z = 3			# Land here 
	setpoints.append(pos7)

	

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
