#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
import numpy
import numpy as np
from math import sqrt
import math
# END IMPORT

# BEGIN STD_MSGS
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
# END STD_MSGS

# BEGIN SRV IMPORT
from std_srvs.srv import SetBool,SetBoolResponse
from jelly_description.srv import numpyArray
from jelly_description.srv import numpyArrayResponse
# END SRV IMPORT
#%$#1
# BEGIN SETUP
rospy.init_node("custom_controller_stab")
update_rate = 50 #Hz
rate = rospy.Rate(update_rate) 
# END SETUP
# rando = Odometry()
# rando.pose.pose.position.x
class PositionAttitudeController():
    def __init__(self):
        #%$#3.1
        # Position control system gains
        self.pos_proportional = 25.9374492277101
        self.pos_integral = 3.63947016352946
        self.pos_derivative = 41.8416925944087
        self.pos_filter = 43.7209955392367
        self.update_time = 1/update_rate
        # Attitude control system gains
        self.q_proportional = 5
        self.omega_proportional = 2
        # Desired position
        #self.reference_position = numpy.array([[0.0],[0.0],[-5]]) Defined in int_diff_eqs
        # Desired attitude (can be changed to take quaternion inputs from trajectory at later date.) w, x, y, z
        #self.reference_attitude = numpy.array([[1.0],[0.0],[0.0],[0.0]])#numpy.array([[0.7071068],[0.0],[0.7071068],[0]])#numpy.array([[0.9659258],[0.0],[-0.258819],[0.0]])#
        # Initialize position controller difference equations
        self.int_diff_eqs()
        self.first_int_diff_eqs()
        # Define publisher and subscriber objects.
        #self.flag_subscriber = rospy.Subscriber("/jelly/main_flag",Int8,self.import_flag)
        self.wrench_publisher = rospy.Publisher("/jelly/controller/command_wrench",WrenchStamped, queue_size=1)
        # self.command_publisher = rospy.Publisher("/jelly/commandZ", Float32, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/rov/depth_odom", Odometry, self.input_position)
        self.quaternion_subscriber = rospy.Subscriber('/imu/data', Imu, self.input_quaternion_orientation)
        
        # Define controller on/off behavior:
        self.set_bool_service = rospy.Service('set_controller_state',SetBool,self.controller_state_callback)
        # Define command position control:
        self.command_pos_service = rospy.Service('set_command_position',numpyArray,self.command_position_callback)
        # Define command attitude control:
        self.command_att_service = rospy.Service('set_command_attitude',numpyArray,self.command_attitude_callback)

        # Define attitude gains change service:
        self.attitude_gains_service = rospy.Service('set_attitude_gains',numpyArray,self.attitude_gains_callback)
        self.position_gains_service = rospy.Service('set_position_gains',numpyArray,self.position_gains_callback)

    def attitude_gains_callback(self,data):
        a = numpyArrayResponse()
        self.q_proportional = data.data[0]
        self.omega_proportional = data.data[1]
        print("New proportional gain is: ")
        print(self.q_proportional)
        print("New velocity gain is: ")
        print(self.omega_proportional)
        a.message = "Receive successful, request received was: " + str(data.data)
        return a

    def position_gains_callback(self,data):
        a = numpyArrayResponse()
        self.pos_proportional = data.data[0]
        self.pos_integral = data.data[1]
        self.pos_derivative = data.data[2]
        self.pos_filter = data.data[3]
        self.int_diff_eqs()
        print("New proportional gain is: ")
        print(self.pos_proportional)
        print("New integral gain is: ")
        print(self.pos_integral)
        print("New derivative gain is: ")
        print(self.pos_derivative)
        print("New filter gain is: ")
        print(self.pos_filter)
        a.message = "Receive successful, request received was: " + str(data.data)
        return a

    def command_attitude_callback(self,data):
        if sqrt(data.data[0]**2 + data.data[1]**2 + data.data[2]**2 + data.data[3]**2) <= 1:
            print("Valid quaternion!")
            self.reference_attitude = np.array([[data.data[0]],[data.data[1]],[data.data[2]],[data.data[3]]]) # w,x,y,z
            print("Command attitude has been changed to: " + str(self.reference_attitude))
        else:
            print("Invalid quaternion! It has not been changed and you're a bad boy >:(")

        a = numpyArrayResponse()
        a.message = "Receive successful, request received was: " + str(data.data)
        return a

    def command_position_callback(self,data):
        self.reference_position = np.array([[data.data[0]],[data.data[1]],[data.data[2]]])
        a = numpyArrayResponse()
        a.message = "Receive successful, request received was: " + str(data.data)
        print("Command position has been changed to: " + str(self.controller_state))
        return a

    def controller_state_callback(self,data):
        a = SetBoolResponse()
        a.message = "Receive successful, request received was: " + str(data.data)
        self.controller_state = data.data
        print("Controller state has been changed to: " + str(self.controller_state))
        return a
    
    def int_diff_eqs(self):
        #%$#3.2
        a = 1 + ((self.pos_filter * self.update_time)/2)
        b = ((self.pos_filter * self.update_time)/2) - 1
        b0 = ((2*self.pos_proportional*a) + (self.pos_integral*self.update_time*a) + (2*self.pos_derivative*self.pos_filter))
        b1 = ((2*self.pos_proportional*(b-a)) + (((self.pos_integral*self.update_time*b)+(self.pos_integral*self.update_time*a))) - (4*self.pos_derivative*self.pos_filter))
        b2 = ((-1*2*self.pos_proportional*b) + (self.pos_integral*self.update_time*b) + (2*self.pos_derivative*self.pos_filter))
        a0 = (2*a)
        a1 = (2*(b-a))
        a2 = (-1*2*b)

        self.diff_eq_coeff = numpy.array([(b0/a0),(b1/a0),(b2/a0),(a1/a0),(a2/a0)])

    def first_int_diff_eqs(self):
        self.command_force = numpy.array([[0.0],[0.0],[0.0]]) # Don't do this, just use numpy.zeros(col,row) to initialize arrays.
        self.command_force_1 = numpy.array([[0.0],[0.0],[0.0]])
        self.command_force_2 = numpy.array([[0.0],[0.0],[0.0]])
        self.error_pos = numpy.array([[0.0],[0.0],[0.0]])
        self.error_pos_1 = numpy.array([[0.0],[0.0],[0.0]])
        self.error_pos_2 = numpy.array([[0.0],[0.0],[0.0]])
        self.reference_position = numpy.array([[0.0],[0.0],[-1.0]])
        self.reference_attitude = numpy.array([[1.0],[0.0],[0.0],[0.0]])
        self.quaternion_vector = numpy.array([[1.0],[0.0],[0.0],[0.0]])
        self.angular_velocity_vector = numpy.array([[0.0],[0.0],[0.0]])
        self.current_position = numpy.array([[0.0],[0.0],[0.0]])
        self.error_att = numpy.array([[0.0],[0.0],[0.0]])
        self.controller_state = False
        print("int_diff_eqs")

    def calculate_error_quaternion(self):
        # self.error[0,0] = (self.reference_attitude[0,0] * self.quaternion_vector[0,0]) + ( -1 * self.reference_attitude[1,0] * self.quaternion_vector[1,0]) + ( -1 * self.reference_attitude[2,0] * self.quaternion_vector[2,0]) + ( -1 * self.reference_attitude[3,0] * self.quaternion_vector[3,0])
        self.error_com_att_1 = (self.reference_attitude[0,0] * self.quaternion_vector[1,0]) + ( 1 * self.reference_attitude[1,0] * self.quaternion_vector[0,0]) + ( 1 * self.reference_attitude[2,0] * self.quaternion_vector[3,0]) + ( -1 * self.reference_attitude[3,0] * self.quaternion_vector[2,0])
        self.error_com_att_2 = (self.reference_attitude[0,0] * self.quaternion_vector[2,0]) + ( -1 * self.reference_attitude[1,0] * self.quaternion_vector[3,0]) + ( 1 * self.reference_attitude[2,0] * self.quaternion_vector[0,0]) + ( 1 * self.reference_attitude[3,0] * self.quaternion_vector[1,0])
        self.error_com_att_3 = (self.reference_attitude[0,0] * self.quaternion_vector[3,0]) + ( 1 * self.reference_attitude[1,0] * self.quaternion_vector[2,0]) + ( -1 * self.reference_attitude[2,0] * self.quaternion_vector[1,0]) + ( 1 * self.reference_attitude[3,0] * self.quaternion_vector[0,0])
        
        self.error_att[0,0] = self.error_com_att_1
        self.error_att[1,0] = self.error_com_att_2
        self.error_att[2,0] = self.error_com_att_3        
        # print("calculate_error_quaternion")

    def input_position(self,msg):
        #print("Input Pos: ")
        self.current_position[0,0] = msg.pose.pose.position.x
        self.current_position[1,0] = msg.pose.pose.position.y
        self.current_position[2,0] = float(msg.pose.pose.position.z)
        #print(self.current_position)
        
        # print("input_position")

    def input_quaternion_orientation(self, msg2):
        # Put vectors into numpy array for further processing.
        #print("Input Orien: ")
        self.quaternion_vector[0,0] = msg2.orientation.w
        self.quaternion_vector[1,0] = msg2.orientation.x
        self.quaternion_vector[2,0] = msg2.orientation.y
        self.quaternion_vector[3,0] = msg2.orientation.z
        # print(self.quaternion_vector)
        self.angular_velocity_vector[0,0] = msg2.angular_velocity.x
        self.angular_velocity_vector[1,0] = msg2.angular_velocity.y
        self.angular_velocity_vector[2,0] = msg2.angular_velocity.z
        #print(self.quaternion_vector)

    def calculate_error_position(self):
        self.error_pos = self.reference_position - self.current_position

    def difference_eq(self):
        #print("Current controller state is: " + str(self.controller_state))
        if self.controller_state == True:
            #print("diff eq")
            self.calculate_error_position()
            #print(self.error_pos)
            self.command_force = (self.diff_eq_coeff[0] * self.error_pos) + (self.diff_eq_coeff[1] * self.error_pos_1) + (self.diff_eq_coeff[2] * self.error_pos_2) - (self.diff_eq_coeff[3] * self.command_force_1) - (self.diff_eq_coeff[4] * self.command_force_2)
            self.error_pos_2 = self.error_pos_1
            self.error_pos_1 = self.error_pos
            self.command_force_2 = self.command_force_1
            self.command_force_1 = self.command_force
            self.calculate_error_quaternion()
            # Apply control law.
            self.command_torque = (-1 * self.q_proportional * self.error_att) + (-1 * self.omega_proportional * self.angular_velocity_vector)
            #print(self.command_torque)
            self.publish_wrench()
            # print("input_quaternion_orientation")
        elif self.controller_state == False:
            self.command_force = numpy.array([[0.0],[0.0],[0.0]])
            self.command_torque = numpy.array([[0.0],[0.0],[0.0]])
            self.publish_wrench()

    def publish_wrench(self):
        # print("publish_wrench_begin")
        # Set message parameters.
        message = WrenchStamped()
        message.header.seq = 10
        message.header.stamp = rospy.Time()
        message.header.frame_id = "base_link"
        message.wrench.force.x = self.command_force[0,0]
        message.wrench.force.y = self.command_force[1,0]
        message.wrench.force.z = self.command_force[2,0]
        message.wrench.torque.x = self.command_torque[0,0]
        message.wrench.torque.y = self.command_torque[1,0]
        message.wrench.torque.z = self.command_torque[2,0]
        # Publish wrench
        self.wrench_publisher.publish(message)

#%$#2

node = PositionAttitudeController()

while not rospy.is_shutdown():

    node.difference_eq()
    rate.sleep()