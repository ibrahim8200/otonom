#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from os import times
import time
from std_msgs.msg import Int16
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()


global range1
range1 = 5.0
global range2
range2 = 5.0
global range3
range3 = 5.0
global range4
range4 = 5.0
global X
global Y
X = None
Y = None


def listener():
    rospy.init_node('otonom1', anonymous=True)


class MyFirstNode():
    def __init__(self):    
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
        def callbackx(msgx):
            global X
            X = msgx.data

        def callbacky(msgy):
            global Y
            Y = msgy.data


        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        while not rospy.is_shutdown():
            rospy.Subscriber("/sonar", Range, callback)
            rospy.Subscriber("/sonar1", Range, callback1)
            rospy.Subscriber("/sonar2", Range, callback2)
            rospy.Subscriber("/sonar3", Range, callback3)
            rospy.Subscriber("/x", Int16, callbackx)
            rospy.Subscriber("/y", Int16, callbacky)

            t = rospy.get_time()
            hz = 0.8
            
            def ileri():
                master.mav.manual_control_send(
                    master.target_system,
                    500,
                    0,
                    0,
                    0,
                    0)

            def arka():
                master.mav.manual_control_send(
                    master.target_system,
                    -500,
                    0,
                    0,
                    0,
                    0)
            def alt():
                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    0,
                    450,
                    0,
                    0)
            def sut():
                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    0,
                    -450,
                    0,
                    0)
            def sag():
                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    400,
                    0,
                    0,
                    0)
            def sol():
                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    -400,
                    0,
                    0,
                    0)
            def sag_d():
                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    0,
                    0,
                    400,
                    0)
            def sol_d():
                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    0,
                    0,
                    -400,
                    0)
            def dur():
                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    0,
                    0,
                    0,
                    0)
            
            
            x = 0
            x1 = 215
            x2 = 430
            x3 = 640
            y = 0
            y1 = 160
            y2 = 320
            y3 = 480


            if X is None and Y is None:
                if range1 > 3.0 and range2 > 3.0 and range3 > 3.0 and range4 > 3.0 :
                    ileri()
                    print("x ve y yok")
                if range1 < 3.0 and range2 > 3.0 and range3 > 3.0 and range4 > 3.0 :
                    sol_d()
                    print("x ve y yok")

                if  range1 > 3.0 and range2 < 3.0 and range3 > 3.0 and range4 > 3.0 :
                    ileri()
                    print("x ve y yok")

                if  range1 > 3.0 and range2 > 3.0 and range3 < 3.0 and range4 > 3.0 :
                    sol_d()
                    print("x ve y yok")

                if  range1 > 3.0 and range2 > 3.0 and range3 > 3.0 and range4 < 3.0 :
                    sag_d()
                    print("x ve y yok")

                if range1 < 3.0 and range2 < 3.0 and range3 > 3.0 and range4 > 3.0 :
                    sol_d()
                    print("x ve y yok")

                if range1 < 3.0 and range2 > 3.0 and range3 < 3.0 and range4 > 3.0 :
                    sol_d()
                    print("x ve y yok")

                if range1 < 3.0 and range2 > 3.0 and range3 > 3.0 and range4 < 3.0 :
                    sag_d()
                    print("x ve y yok")

                if  range1 > 3.0 and range2 < 3.0 and range3 < 3.0 and range4 > 3.0 :
                    sol_d()
                    print("x ve y yok")

                if  range1 > 3.0 and range2 < 3.0 and range3 < 3.0 and range4 < 3.0 :
                    sag_d()
                    print("x ve y yok")

                if  range1 > 3.0 and range2 > 3.0 and range3 < 3.0 and range4 < 3.0 :
                    ileri()
                    print("x ve y yok")

                if  range1 > 3.0 and range2 < 3.0 and range3 < 3.0 and range4 < 3.0 :
                    ileri()
                    print("x ve y yok")

                if  range1 < 3.0 and range2 > 3.0 and range3 < 3.0 and range4 < 3.0 :
                    arka()
                    print("x ve y yok")

                if  range1 < 3.0 and range2 < 3.0 and range3 > 3.0 and range4 < 3.0 :
                    sag_d()
                    print("x ve y yok")

                if  range1 < 3.0 and range2 < 3.0 and range3 < 3.0 and range4 > 3.0 :
                    sol_d()
                    print("x ve y yok")

                if  range1 < 3.0 and range2 < 3.0 and range3 < 3.0 and range4 < 3.0 :
                    dur()
                    print("x ve y yok")


            if X > 0 and Y > 0:
                if x < X <x1 and y < Y < y1 :
                    ileri()
                    print("ileri")
                if x1 < X <x2 and y < Y < y1 :
                    ileri()
                    print("ileri")
                if x2 < X <x3 and y < Y < y1 :
                    ileri()
                    print("ileri")

                if x < X <x1 and y1 < Y < y2 :
                    sol()
                    print("sol")
                                        
                if x1 < X <x2 and y1 < Y < y2 :
                    alt()
                    print("alt")     
                               
                if x2 < X <x3 and y1 < Y < y2 :
                    sag()
                    print("sag")                    
                if x < X <x1 and y2 < Y < y3 :
                    arka()
                    print("arka")                    
                if x1 < X <x2 and y2 < Y < y3 :
                    arka()
                    print("arka")  
                if x2 < X <x3 and y2 < Y < y3 :
                    arka()
                    print("arka")  

            print ("x :" + str(X) + "Y : "+ str(Y))



if __name__ == '__main__':
    listener()
    node = MyFirstNode()
