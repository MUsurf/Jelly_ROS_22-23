#!/usr/bin/env python3

# BEGIN IMPORT
import rospy
# END IMPORT

# BEGIN STD_MSGS
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import roslaunch
import time
from std_msgs.msg import Int32MultiArray
# END STD_MSGS

rospy.init_node("stab_main")
rate = rospy.Rate(50)

class stabMain():
    def __init__(self):
        print("Test")
        self.start_time = rospy.get_time()
        self.main_command_publisher = rospy.Publisher("/jelly/main/command_wrench",WrenchStamped,queue_size=10)
        self.main_pose_publisher = rospy.Publisher("/jelly/main/pose_gt",Odometry,queue_size=10)
        self.main_quaternion_publisher = rospy.Publisher("/jelly/main/orientation",Imu,queue_size=10)
        self.flag_publisher = rospy.Publisher("/jelly/main_flag",Int8,queue_size=10)
        self.desired_pose_publisher = rospy.Publisher("/jelly/command_pose",Pose,queue_size=10)
        self.add_wrench_pub = rospy.Publisher('/jelly/add_wrench',WrenchStamped,queue_size=10)
        self.publishers = rospy.Publisher('/command',Int32MultiArray,queue_size=1)
        self.flag = Int8()
        self.start_esc = False
        # self.create_main_command_wrench()
        # self.create_main_pose()
        # self.create_main_attitude()
        # self.create_desired_pose_1()

    # def create_main_command_wrench(self):
    #     self.main_command_wrench = WrenchStamped()
    #     self.main_command_wrench.header.seq = 0
    #     self.main_command_wrench.header.frame_id = "id_1"
    #     self.main_command_wrench.wrench.force.x = 0
    #     self.main_command_wrench.wrench.force.y = 0
    #     self.main_command_wrench.wrench.force.z = 0
    #     self.main_command_wrench.wrench.torque.x = 0
    #     self.main_command_wrench.wrench.torque.y = 0
    #     self.main_command_wrench.wrench.torque.z = 0

    # def create_main_pose(self):
    #     self.main_pose = Odometry()
    #     self.main_pose.pose.pose.position.x = 0
    #     self.main_pose.pose.pose.position.y = 0
    #     self.main_pose.pose.pose.position.z = 0
    
    # def create_main_attitude(self):
    #     self.main_attitude = Imu()
    #     self.main_attitude.orientation.x = 0
    #     self.main_attitude.orientation.y = 0
    #     self.main_attitude.orientation.z = 0
    #     self.main_attitude.orientation.w = 1

    def create_desired_pose_1(self):
        self.desired_pose = Pose()
        self.desired_pose.position.x = 0
        self.desired_pose.position.y = 0
        self.desired_pose.position.z = 0
        self.desired_pose.orientation.x = 0
        self.desired_pose.orientation.y = 0
        self.desired_pose.orientation.z = 0
        self.desired_pose.orientation.w = 1

    def create_desired_pose_2(self):
        self.desired_pose.position.x = 0
        self.desired_pose.position.y = 0
        self.desired_pose.position.z = -0.6
        self.desired_pose.orientation.x = 0
        self.desired_pose.orientation.y = 0
        self.desired_pose.orientation.z = 0
        self.desired_pose.orientation.w = 1

    def check_time(self):
        # print("Check time")
        # print(rospy.get_time())
        while not rospy.is_shutdown():
            #self.main_command_wrench.header.stamp = rospy.Time()
            # self.main_pose.header.stamp = rospy.Time()
            # self.main_attitude.header.stamp = rospy.Time()
            #self.main_command_publisher.publish(self.main_command_wrench)
            print("Self Main Pose: ")
            #print(self.main_pose)
            # self.main_pose_publisher.publish(self.main_pose)
            # self.main_quaternion_publisher.publish(self.main_attitude)
            print(rospy.get_time() - self.start_time)
            if (rospy.get_time() - self.start_time) < 1:
                # print("0")
                # control_msg = Int32MultiArray()
                # control_msg.data = [0,1950,1950,0,0,1950,1950,0]
                self.create_desired_pose_1()
                self.flag.data = 0
                self.desired_pose_publisher.publish(self.desired_pose)
                self.flag_publisher.publish(self.flag)
                # if self.start_esc is False:
                #     node_motor = 'motor_commander'
                #     motor_commander_node = roslaunch.core.Node(package='motor_command',node_type='motor_commander.py',name=node_motor,output='screen')
                #     launch = roslaunch.scriptapi.ROSLaunch()
                #     launch.start()
                #     launch.launch(motor_commander_node)
                #     time.sleep(15)
                #     self.start_esc = True
                #self.publishers.publish(control_msg)
            elif 1 <= (rospy.get_time() - self.start_time) <= 20:
                if self.flag.data == 0:
                    pkg = 'main'
                    #print(type(pkg))
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    node_control = 'custom_controller_stab_hard'
                    #print(type(node_control))
                    node_thrust = 'custom_thrust_allocator_stab_hard'
                    depth_node = roslaunch.core.Node(package='ms5837',node_type='ms5837_ros.py',name='ms5837_node',output='screen')
                    launch.launch(depth_node)
                    rospy.wait_for_message('/rov/depth_odom',Odometry,timeout=20)
                    #print(type(node_thrust))
                    controller_node = roslaunch.core.Node(package=pkg,node_type='custom_controller_stab_hard_fix.py',name=node_control,output='screen')
                    #print(type(controller_node))
                    thrust_allocator_node = roslaunch.core.Node(package=pkg,node_type='custom_thrust_allocator_stab_hard.py',name=node_thrust,output='screen') 
                    launch.launch(controller_node)
                    launch.launch(thrust_allocator_node)
                    # try:
                    #     print("INT TRY")
                    #     depth_pkg = 'ms5837'
                    #     depth_node_1 = 'ms5837_ros'
                    #     depth_node_2 = 'ms5837_variance'
                    #     depth_node_1_ob = roslaunch.core.Node(package=depth_pkg,node_type='ms5837_ros.py',name=depth_node_1)
                    #     depth_node_2_ob = roslaunch.core.Node(package=depth_pkg,node_type='ms5837_variance.py',name=depth_node_2)
                    #     launch.launch(depth_node_1_ob)
                    #     launch.launch(depth_node_2_ob)
                    #     self.depth_sensor = True
                    # except:
                        # self.depth_sensor = False
                # if self.depth_sensor:
                print("1")
                self.flag.data = 1
                self.create_desired_pose_2()
                self.desired_pose_publisher.publish(self.desired_pose)
                self.flag_publisher.publish(self.flag)
                # else:
                #     self.add_wrench = WrenchStamped()
                #     self.add_wrench.wrench.force.x = 0
                #     self.add_wrench.wrench.force.y = 0
                #     self.add_wrench.wrench.force.z = 40
                #     self.add_wrench_pub.publish(self.add_wrench)
            elif (rospy.get_time() - self.start_time) >= 20:
                self.flag.data = 2
                self.create_desired_pose_2()
                self.desired_pose_publisher.publish(self.desired_pose)
                self.flag_publisher.publish(self.flag)
                # print("TO 20")
                # control_msg = Int32MultiArray()
                # control_msg.data = [1950,0,0,1950,1950,0,0,1950]
                # self.publishers.publish(control_msg)
                # self.desired_pose_publisher.publish(self.desired_pose)
                # self.add_wrench = WrenchStamped()
                # self.add_wrench.wrench.force.x = 100
                # self.add_wrench.wrench.force.y = 0
                # self.add_wrench.wrench.force.z = 0
                # self.add_wrench_pub.publish(self.add_wrench)
            rate.sleep()

node = stabMain()

while not rospy.is_shutdown():

    node.check_time()
    rate.sleep()

