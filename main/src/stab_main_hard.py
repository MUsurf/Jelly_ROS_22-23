#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
import roslaunch
# END IMPORT

# BEGIN STD_MSGS
import numpy
import roslaunch
# END STD_MSGS

# BEGIN SRV IMPORT
from std_srvs.srv import SetBool
from jelly_description.srv import numpyArray
# END SRV IMPORT

rospy.init_node("stab_main")
rate = rospy.Rate(50)

# The main function will be the service client. We want our first service to turn the controller on and off, for example.
# Another service will allow us to give an input pose and have that sent to the controller.
# A third service will allow a heading mode and will convert the input heading to a set of quaternion inputs.
print("Waiting for controller state service")
rospy.wait_for_service('set_controller_state')
print("Waiting for allocator state service")
rospy.wait_for_service('set_allocator_state')
print("Waiting for command position service")
rospy.wait_for_service('set_command_position')
print("Waiting for command attitude service")
rospy.wait_for_service('set_command_attitude')
print("Waiting for attitude gain service.")
rospy.wait_for_service('set_attitude_gains')
print("Waiting for position gain service.")
rospy.wait_for_service('set_position_gains')
# THis is comment
# Create service proxy functions.

try:
    controller_on_off = rospy.ServiceProxy('set_controller_state',SetBool)
    # controller_on_off(True) / controller_on_off(False)

    allocator_on_off = rospy.ServiceProxy('set_allocator_state',SetBool)
    # allocator_on_off(True) / allocator_on_off(False)

    command_position_serv_prox = rospy.ServiceProxy('set_command_position',numpyArray)
    # command_position_numpy_array = numpy.array([[1.0],[1.0],[-5.0]])
    # response = command_position_serv_prox([command_position_numpy_array[0,0],command_position_numpy_array[1,0],command_position_numpy_array[2,0]])

    command_attitude_serv_prox = rospy.ServiceProxy('set_command_attitude',numpyArray)
    # command_attitude_numpy_array = numpy.array([[1.0],[0.0],[0.0],[0.0]]) # w,x,y,z
    # response_att = command_attitude_serv_prox([command_attitude_numpy_array[0,0],command_attitude_numpy_array[1,0],command_attitude_numpy_array[2,0],command_attitude_numpy_array[3,0]])

    # The controller will check that the provided quaternions are valid before applying them! It will default to the previous attitude reference if this is the case.

    attitude_gain_serv_prox = rospy.ServiceProxy('set_attitude_gains',numpyArray)
    # new_attitude_gains = numpy.array([[-505.5],[100.5]]) # These gains are unstable but useful for testing.
    # attitude_gain_serv_prox([new_attitude_gains[0,0],new_attitude_gains[1,0]])

    position_gain_serv_prox = rospy.ServiceProxy('set_position_gains',numpyArray)
    # new_position_gains = numpy.array([[25.5],[4000],[42],[44]]) # P, I, D, N These gains are unstable but useful for testing.
    # response_gain_1 = position_gain_serv_prox([new_position_gains[0,0],new_position_gains[1,0],new_position_gains[2,0],new_position_gains[3,0]])

except rospy.ServiceException as e:
    print("Service proxy creation failed.")

# Test of each proxy service.

response_1 = controller_on_off(True)

allocator_on_off(True) # Test of calling without intersecting message.

command_position_numpy_array = numpy.array([[1.0],[1.0],[-5.0]])
response = command_position_serv_prox([command_position_numpy_array[0,0],command_position_numpy_array[1,0],command_position_numpy_array[2,0]])

command_attitude_numpy_array = numpy.array([[1.0],[0.0],[0.0],[0.0]]) # w,x,y,z
response_att = command_attitude_serv_prox([command_attitude_numpy_array[0,0],command_attitude_numpy_array[1,0],command_attitude_numpy_array[2,0],command_attitude_numpy_array[3,0]])

new_attitude_gains = numpy.array([[-505.5],[100.5]]) # These gains are unstable but useful for testing.
#attitude_gain_serv_prox([new_attitude_gains[0,0],new_attitude_gains[1,0]])

new_position_gains = numpy.array([[25.5],[4000],[42],[44]]) # P, I, D, N These gains are unstable but useful for testing.
#response_gain_1 = position_gain_serv_prox([new_position_gains[0,0],new_position_gains[1,0],new_position_gains[2,0],new_position_gains[3,0]])

#print(response)
#print(response_2)