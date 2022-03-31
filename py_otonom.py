#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from mavros_msgs.msg import MotorSetpoint
from os import times
from pymavlink import mavutil

global range1
range1 = 5
global range2
range2 = 5
global range3
range3 = 5
global range4
range4 = 5
global x1
global x2
global x3
global x4

x1 = 0
x2 = 0
x3 = 0
x4 = 0

def callback(msg1):
    global range1
    range1 = msg1.range
    range1 = round(range1 ,1)
    return range1

def callback1(msg2):
    global range2
    range2 = msg2.range
    range2 = round(range2 ,1)
    return range2

def callback2(msg3):
    global range3
    range3 = msg3.range
    range3 = round(range3 ,1)
    return range3

def callback3(msg4):
    global range4
    range4 = msg4.range
    range4 = round(range4 ,1)
    return range4


def listener():
    rospy.init_node('sonar_test', anonymous=True)
class MyFirstNode():

    def __init__(self):    
        self.setpoint_pub = rospy.Publisher("mavros/setpoint_motor/setpoint",MotorSetpoint,queue_size=1)
    def run(self):
        rate = rospy.Rate(30.0)
        master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        master.wait_heartbeat()
        while not rospy.is_shutdown():
            rospy.Subscriber("bluerov/sonar", Range, callback)
            rospy.Subscriber("bluerov/sonar1", Range, callback1)
            rospy.Subscriber("bluerov/sonar2", Range, callback2)
            rospy.Subscriber("bluerov/sonar3", Range, callback3)
            msg = MotorSetpoint()
            msg.header.stamp = rospy.Time.now()
            t = rospy.get_time()                    
            if  range1 < 2.5:
                x1 = 500
                x2 = 500
                x3 = 0
                x4 = 0

            
            if  range2 < 2.5:
                x1 = 500
                x2 = 0
                x3 = 500
                x4 = 0

            
            if  range3 < 2.5:
                x1 = 0
                x2 = 0
                x3 = 500
                x4 = 500
 
                        
            if  range3 < 2.5:
                x1 = 0
                x2 = 500
                x3 = 0
                x4 = 500

           
            master.mav.manual_control_send(
                master.target_system,
                x1,
                x2,
                x3,
                x4,
                0)
            self.setpoint_pub.publish(msg)
            print(range1)
            print(range2)
            print(range3)
            print(range4)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0)

            # wait until disarming confirmed
        master.motors_disarmed_wait()


if __name__ == '__main__':
    listener()
    node = MyFirstNode()
    node.run()


