#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
from kv_host_link import *

class RobotCommunication:
    def __init__(self, v_l, v_r):
        self.v_l = v_l
        self.v_r = v_r

    def callback_velocity_command(self,msg):
        rospy.loginfo("Message '{}' recieved".format(msg.data))
        self.v_l = msg.data[0]
        self.v_r = msg.data[1]
        # rospy.loginfo("v_l: '{}' ".format(self.v_l))
        # rospy.loginfo("VELOCITY_LEFT : {}".format(msg.data[0]))
        # rospy.loginfo("VELOCITY_RIGHT : {}".format(msg.data[1]))
        rc.write_robot()
        

    def write_robot(self):
        if not kv.write_plc(VELOCITY_LEFT, self.v_l):
            rospy.loginfo("VL command sent.")
        else:
            rospy.logwarn("VL command sent error")

        if not kv.write_plc(VELOCITY_RIGHT, self.v_r):
            rospy.loginfo("VR command sent.")
        else:
            rospy.logwarn("VR command sent error")



    def robot_com(self):

        rospy.init_node('robot_communication', anonymous=True)

        rospy.Subscriber("/cmd_vel", Int16MultiArray, rc.callback_velocity_command)
        
        if not kv.connect_plc():
            rospy.loginfo("Connection to PLC successful.")

        else:
            rospy.logwarn("PLC connection error!")

        rospy.spin()

        # rate = rospy.Rate(20) # ROS Rate at 20Hz

        # while not rospy.is_shutdown():
        #     rc.write_robot()
        #     rate.sleep()
        self.v_l=0
        self.v_r=0
        rc.write_robot()
        kv.close_plc()


if __name__ == '__main__':
    kv = KvHostLink()
    rc = RobotCommunication(0, 0)
    rc.robot_com()

