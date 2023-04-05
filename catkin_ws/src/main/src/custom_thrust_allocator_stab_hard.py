#!/usr/bin/env python3

# BEGIN IMPORT
from cmath import sqrt
import numpy
import rospy
import time
# END IMPORT

# BEGIN MSGS
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
# END MSGS

# BEGIN SETUP
rospy.init_node("custom_thrust_allocator_stab")
rate = rospy.Rate(50)
# END SETUP

class ThrustAllocator():

    def __init__(self):
        self.input_values = [1150, 1154, 1158, 1162, 1166, 1170, 1174, 1178, 1182, 1186, 1190, 1194, 1198, 1202, 1206, 1210, 1214, 1218, 1222, 1226, 1230, 1234, 1238, 1242, 1246, 1250, 1254, 1258, 1262, 1266, 1270, 1274, 1278, 1282, 1286, 1290, 1294, 1298, 1302, 1306, 1310, 1314, 1318, 1322, 1326, 1330, 1334, 1338, 1342, 1346, 1350, 1354, 1358, 1362, 1366, 1370, 1374, 1378, 1382, 1386, 1390, 1394, 1398, 1402, 1406, 1410, 1414, 1418, 1422, 1426, 1430, 1434, 1438, 1442, 1446, 1450, 1454, 1458, 1462, 1466, 1470, 1474, 1478, 1482, 1486, 1490, 1494, 1498, 1502, 1506, 1510, 1514, 1518, 1522, 1526, 1530, 1534, 1538, 1542, 1546, 1550, 1554, 1558, 1562, 1566, 1570, 1574, 1578, 1582, 1586, 1590, 1594, 1598, 1602, 1606, 1610, 1614, 1618, 1622, 1626, 1630, 1634, 1638, 1642, 1646, 1650, 1654, 1658, 1662, 1666, 1670, 1674, 1678, 1682, 1686, 1690, 1694, 1698, 1702, 1706, 1710, 1714, 1718, 1722, 1726, 1730, 1734, 1738, 1742, 1746, 1750, 1754, 1758, 1762, 1766, 1770, 1774, 1778, 1782, 1786, 1790, 1794, 1798, 1802, 1806, 1810, 1814, 1818, 1822, 1826, 1830, 1834, 1838, 1842, 1846, 1850, 1854, 1858, 1862, 1866, 1870, 1874, 1878, 1882, 1886, 1890, 1894, 1898, 1902, 1906, 1910, 1914, 1918, 1922, 1926, 1930, 1934, 1938, 1942, 1946, 1950]
        self.output_values_forw_T200 = [-39.92156178, -39.73615605, -39.4691718, -38.89070592, -38.26774267, -37.95626105, -37.51128729, -37.28880042, -36.79932929, -36.44335029, -35.86488441, -35.19742378, -34.52996316, -33.86250253, -33.32853402, -32.4830839, -31.90461802, -31.41514689, -30.48070201, -29.81324138, -29.36826763, -28.83429913, -28.07784375, -27.67736737, -27.18789625, -26.56493299, -26.07546187, -25.31900649, -25.00752486, -24.60704849, -24.07307998, -23.36112198, -23.09413773, -22.33768235, -21.93720597, -21.5367296, -20.78027422, -20.42429522, -19.84582934, -19.40085559, -19.04487659, -18.24392383, -17.79895008, -17.30947895, -16.6865157, -16.37503407, -15.7965682, -15.26259969, -14.86212332, -14.59513907, -14.10566794, -13.70519156, -13.21572043, -12.77074668, -12.32577293, -11.79180443, -11.39132805, -10.99085167, -10.81286217, -10.32339105, -10.01190942, -9.611433043, -9.255454042, -8.854977665, -8.543496038, -8.054024911, -7.69804591, -7.297569533, -7.030585282, -6.67460628, -6.363124654, -6.051643027, -5.695664026, -5.295187649, -5.028203398, -4.716721771, -4.36074277, -4.093758518, -3.782276892, -3.470795266, -3.159313639, -2.892329388, -2.536350386, -2.31386351, -2.046879259, -1.779895008, -1.468413382, -1.245926506, -1.02343963, -0.845450129, -0.667460628, -0.489471127, -0.355979002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.400476377, 0.533968502, 0.756455378, 0.978942254, 1.245926506, 1.512910757, 1.779895008, 2.13587401, 2.447355636, 2.847832013, 3.159313639, 3.515292641, 3.915769018, 4.271748019, 4.627727021, 5.028203398, 5.47317715, 5.918150902, 6.274129903, 6.67460628, 7.075082657, 7.609051159, 8.009527536, 8.543496038, 8.89947504, 9.299951417, 9.744925169, 10.14540155, 10.7683648, 11.21333855, 11.61381493, 12.14778343, 12.54825981, 13.03773093, 13.61619681, 14.10566794, 14.55064169, 15.08461019, 15.57408132, 16.19704457, 16.59752095, 17.26498158, 17.88794483, 18.42191333, 18.95588184, 19.53434771, 20.06831622, 20.82477159, 21.3587401, 21.75921647, 22.33768235, 23.36112198, 23.85059311, 24.69604324, 25.27450911, 26.03096449, 26.78741987, 27.05440412, 27.85535688, 28.344828, 29.19027813, 29.94673351, 30.48070201, 31.10366526, 31.54863902, 32.39408915, 33.10604715, 33.5510209, 34.4409684, 35.37541328, 36.13186866, 36.66583716, 37.46678992, 38.13425055, 38.80171117, 39.8251508, 40.67060093, 41.73853794, 42.22800906, 42.98446444, 44.23039095, 44.40838045, 45.60980958, 46.18827546, 46.94473084, 47.43420196, 48.36864684, 49.1695996, 49.79256285, 50.46002348, 50.7715051, 51.21647886, 51.45379819]
        self.output_values_back_T200 = []
        for i in self.output_values_forw_T200:
                self.output_values_back_T200.insert(0,-i)
        self.output_values_forw_T100 = []
        for j in  self.output_values_forw_T200:
                self.output_values_forw_T100.append(j/2.2231)
        # tam = rospy.get_param('tam')
        self.motorNum = 8
        self.configuration_matrix = self.calculate_TAM()
        #print(self.configuration_matrix)
        self.inverse_configuration_matrix = None
        if self.configuration_matrix is not None:
            self.inverse_configuration_matrix = numpy.linalg.pinv(self.configuration_matrix)
        self.force_inertial = numpy.zeros([3,1])
        self.torque_body = numpy.zeros([3,1])
        self.DCM = numpy.zeros((3,3))
        self.PWM_data = []
        for lol in range(0,self.motorNum):
            self.PWM_data.append(0)

        # self.topic_names = ['/jelly/thrusters/0/input','/jelly/thrusters/1/input','/jelly/thrusters/2/input','/jelly/thrusters/3/input','/jelly/thrusters/4/input','/jelly/thrusters/5/input','/jelly/thrusters/6/input','/jelly/thrusters/7/input']
        self.flag = 1
        self.publishers = rospy.Publisher('/command',Int32MultiArray,queue_size=1)
        self.add_wrench = numpy.zeros([3,1])
        self.added_wrench = rospy.Subscriber('/jelly/add_wrench',WrenchStamped,self.import_add_wrench)
        #self.flag_subscriber = rospy.Subscriber("/jelly/main_flag",Int8,self.import_flag)
        self.quaternion_subscriber = rospy.Subscriber('/imu/data', Imu, self.input_quaternion_orientation)
        #self.main_wrench_subscriber = rospy.Subscriber('/jelly/main/command_wrench', WrenchStamped,self.input_main_wrench)
        self.wrench_subscriber = rospy.Subscriber('/jelly/controller/command_wrench', WrenchStamped, self.input_wrench)

    # def input_main_wrench(self,msg):
    #     if self.flag == 0:
    #         self.input_wrench(msg)

    # def input_real_wrench(self,msg):
    #     if self.flag == 1:
    #         self.input_wrench(msg)

    def import_add_wrench(self,msg):
        self.add_wrench[0,0] = msg.wrench.force.x
        self.add_wrench[1,0] = msg.wrench.force.y
        self.add_wrench[2,0] = msg.wrench.force.z

    # def import_flag(self,msg):
    #     #print("Flag")
    #     self.flag = msg.data
    #     #print(self.flag)

    def input_quaternion_orientation(self, msg):
        self.x_sqd = msg.orientation.x ** 2
        self.y_sqd = msg.orientation.y ** 2
        self.z_sqd = msg.orientation.z ** 2
        self.w_sqd = msg.orientation.w ** 2
        self.x_y = msg.orientation.x * msg.orientation.y
        self.w_z = msg.orientation.w * msg.orientation.z
        self.x_z = msg.orientation.x * msg.orientation.z
        self.w_y = msg.orientation.w * msg.orientation.y
        self.y_z = msg.orientation.y * msg.orientation.z
        self.w_x = msg.orientation.w * msg.orientation.x
        # self.component[row][column]
        self.DCM[0,0] = (self.w_sqd) + (self.x_sqd) + (-1 * self.y_sqd) + (-1 * self.z_sqd)
        self.DCM[0,1] = (2 * ((self.x_y) + (self.w_z)))
        self.DCM[0,2] = (2 * ((self.x_z) - (self.w_y)))
        self.DCM[1,0] = (2 * ((self.x_y) - (self.w_z)))
        self.DCM[1,1] = (self.w_sqd) + (-1 * self.x_sqd) + (self.y_sqd) + (-1 * self.z_sqd)
        self.DCM[1,2] = (2 * ((self.y_z) + (self.w_x)))
        self.DCM[2,0] = (2 * ((self.x_z) + (self.w_y)))
        self.DCM[2,1] = (2 * ((self.y_z) - (self.w_x)))
        self.DCM[2,2] = (self.w_sqd) + (-1 * self.x_sqd) + (-1 * self.y_sqd) + (self.z_sqd)
        #print(self.DCM)

    def input_wrench(self, msg1):
        #print("IN wrench")
        self.force_inertial = numpy.array([[msg1.wrench.force.x], [msg1.wrench.force.y], [msg1.wrench.force.z]])
        self.torque_inertial = numpy.array([[msg1.wrench.torque.x], [msg1.wrench.torque.y], [msg1.wrench.torque.z]])
        self.force_body = numpy.dot(self.DCM, self.force_inertial) + self.add_wrench# numpy.linalg.inv(self.DCM)
        self.torque_body = self.torque_inertial
        self.both = numpy.concatenate((self.force_body,self.torque_body))

        self.thruster_forces = numpy.dot(self.inverse_configuration_matrix,self.both)
        self.publish_thruster_forces()

    def publish_thruster_forces(self):
        self.message = Int32MultiArray()
        self.message.layout.dim = []
        #self.message.layout.dim.size = 8
        #self.message.layout.dim.stride = 0
        self.message.layout.data_offset = 0
        # self.message.header.stamp = rospy.Time()
        for j in range(0,self.motorNum):
            self.PWM_data[j] = int(self.get_input_value(self.thruster_forces[j],j))
        #print(self.PWM_data)
        # for k in range(len(self.thruster_forces)):
            # self._frame_id = self.message.layout.dim.label[k]
        self.message.data = self.PWM_data
        self.publishers.publish(self.message)

    def get_input_value(self, force,thruster_id):
        if thruster_id == 0 or thruster_id==6:
            #print(thruster_id)
            active = self.output_values_back_T200
            #print(active)
        elif thruster_id == 1 or thruster_id==2 or thruster_id == 5 or thruster_id==4:
            #print(thruster_id)
            active = self.output_values_forw_T100
            #print(active)
        else:
            #print(thruster_id)
            active = self.output_values_forw_T100
            #print(active)
        if force < active[0]:
            self.thruster_input = 1150
            print("Too low")
            return self.thruster_input
        elif force > active[len(active) - 1]:
            self.thruster_input = 1950
            print("Too high")
            return self.thruster_input
        else:
            #l = 0
            for l in range(0,len(self.input_values)):
                if force < active[l]:
                    output_inter_high = active[l]
                    output_inter_low = active[l-1]
                    break
                #l+=1
        input_inter_high = self.input_values[l]
        input_inter_low = self.input_values[l-1]

        return numpy.interp(force,[output_inter_low,output_inter_high],[input_inter_low,input_inter_high])

    def calculate_TAM(self):
        x_axis_normal_vectors = numpy.array([[1/sqrt(2),1/sqrt(2),0],[0,0,-1],[0,0,-1],[1/sqrt(2),-1/sqrt(2),0],[1/sqrt(2),1/sqrt(2),0],[0,0,-1],[0,0,-1],[1/sqrt(2),-1/sqrt(2),0]])
        position_vectors = numpy.array([[0.16931,-0.13868,-0.01801],[0.07044,-0.17671,0.07019],[-0.06964,-0.17671,0.07019],[-0.16848,-0.1387,-0.01801],[-0.16197,0.1428,-0.01801],[-0.06964,0.17529,0.07019],[0.07044,0.17529,0.07019],[0.16931,0.13726,-0.01801]])

        #x_axis_normal_vectors = numpy.array([[1/sqrt(2),1/sqrt(2),0],[0,0,-1],[0,0,-1],[1/sqrt(2),1/sqrt(2),0],[0,0,-1],[0,0,-1],[1/sqrt(2),-1/sqrt(2),0]])
        #position_vectors = numpy.array([[0.16931,-0.13868,-0.01801],[0.07044,-0.17671,0.07019],[-0.06964,-0.17671,0.07019],[-0.16197,0.1428,-0.01801],[-0.06964,0.17529,0.07019],[0.07044,0.17529,0.07019],[0.16931,0.13726,-0.01801]])

        force_coefficients = numpy.zeros((3,self.motorNum))

        torque_coefficients = numpy.zeros((3,self.motorNum))

        for i in range(0,self.motorNum):
            force_coefficients[:,[i]]=numpy.transpose(numpy.atleast_2d(x_axis_normal_vectors[i]))

        for j in range(0,self.motorNum):
            torque_coefficients[:,[j]]=numpy.transpose(numpy.atleast_2d(numpy.cross(position_vectors[j],x_axis_normal_vectors[j])))

        return numpy.concatenate((force_coefficients,torque_coefficients), axis=-2)

if __name__ == '__main__':

    try:
        node = ThrustAllocator()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Attitude::Exception')
    print('Leaving Controller')

# position_vectors = numpy.array([[0.16931,-0.13868,-0.01801],[0.07044,-0.17671,0.07019],[-0.06964,-0.17671,0.07019],[-0.16848,-0.1387,-0.01801],[-0.16197,0.1428,-0.01801],[-0.06964,0.17529,0.07019],[0.07044,0.17529,0.07019],[0.16931,0.13726,-0.01801]])
# Old TAM
# numpy.array([[0.7071054825112364,0.0,0.0,0.7071054825112364,0.7071054825112364,0.0,0.0,0.7071054825112364],[0.7071080798594737,0.0,0.0,-0.7071080798594735,0.7071080798594737,0.0,0.0,-0.7071080798594735],[0.0,0.9999999999932538,0.9999999999932537,0.0,0.0,0.9999999999932538,0.9999999999932541,0.0],[0.012735016518269122,-0.17670999999880788,-0.17670999999880785,-0.01273501651826912,0.012735016518269122,0.17528999999881748,0.1752899999988175,-0.012735016518269122],[-0.012734969740027368,-0.07044025782179103,0.06963974217726399,-0.012734969740027368,-0.012734969740027368,0.069639742177264,-0.07044025782179103,-0.012734969740027371],[0.21778185731566574,0.0,0.0,0.21720909971903257,-0.2155049585974435,0.0,0.0,-0.21677776753049977]])
