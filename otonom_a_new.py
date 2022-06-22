import cv2  # opencv kütüphanesi dahil etme
import numpy as np  # Numpy kütüphanesi dahil etme
import sys
import time

import pymavlink.mavlink
from numpy.core.numeric import tensordot

from pymavlink import mavutil

cap = cv2.VideoCapture(0)  # Kamera aktif hale gelir
za = 0.3
x = 0
x1 = 230
x2 = 320
x3 = 410
x4 = 640
y = 0
y1 = 160
y2 = 240
y3 = 330
y4 = 480

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Choose a mode
mode = 'STABILIZE'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
while True:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

# Arm
master.arducopter_arm()
master.motors_armed_wait()

frame_width = int(vid.get(3))
frame_height = int(vid.get(4))

size = (frame_width, frame_height)

# Below VideoWriter object will create
# a frame of above defined The output
# is stored in 'filename.avi' file.
result = cv2.VideoWriter('filename.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

while 1:  # aşağıdaki işlemleri sonsuza tekrarlansın
    time.sleep(0.2)

    ret, frame = cap.read()                               
    frame = cv2.flip(frame , 1)                            
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)           

    lower1 = np.array([115, 17, 76])
    upper1 = np.array([179, 255, 255])
 
    lower2 = np.array([124,0,0])
    upper2 = np.array([179,255,255])

    kernel = np.ones((5, 5), np.uint8) 
        
    lower_mask1 = cv2.inRange(hsv, lower1, upper1)  
    upper_mask2 = cv2.inRange(hsv, lower2, upper2) 

    mask_Fainal = lower_mask1 + upper_mask2
    mask_Fainal = cv2.erode(mask_Fainal, kernel)
    
    lower_mask1 = cv2.bitwise_and(frame, frame, mask=lower_mask1)
    upper_mask2 = cv2.bitwise_and(frame, frame, mask=upper_mask2)

    contours, _ = cv2.findContours(mask_Fainal, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    detections = []

    for cnt in contours:           
        area = cv2.contourArea(cnt) 
        
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True) 
        x = approx.ravel()[0] 
        y = approx.ravel()[1] 

        if area > 400:      
            x1, y1, w, h = cv2.boundingRect(cnt)                 
            detections.append([x1, y1, w, h]) 

            if 8 <= len(approx) < 9:

                M = cv2.moments(cnt) 
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                cv2.circle(frame, (cx, cy), j, (0, 255, 0), 2)


        if x < cx < x1 and y < cy < y1:  # ust sol
            master.mav.manual_control_send(
                master.target_system,
                200,
                -200,
                400,
                0,
                0)
            time.sleep(za)
        if x1 < cx < x3 and y < cy < y1:  # ust orta
            master.mav.manual_control_send(
                master.target_system,
                200,
                0,
                400,
                0,
                0)
            time.sleep(za)
        if x3 < cx < x4 and y < cy < y1:  # ust sag
            master.mav.manual_control_send(
                master.target_system,
                200,
                200,
                400,
                0,
                0)
            time.sleep(za)
        if x < cx < x1 and y1 < cy < y3:  # orat sol
            master.mav.manual_control_send(
                master.target_system,
                0,
                -200,
                400,
                0,
                0)
            time.sleep(za)

        if x1 < cx < x3 and y1 < cy < y3:  # orat orta
            master.mav.manual_control_send(
                master.target_system,
                0,
                0,
                450,
                0,
                0)
            time.sleep(0.3)

            if x1 < cx < x3 and y1 < cy< y3:
                if x1 < cx < x2 and y1 < cy < y2:
                    master.mav.manual_control_send(
                        master.target_system,
                        100,
                        -100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if x2 < cx < x3 and y1 < cy < y2:
                    master.mav.manual_control_send(
                        master.target_system,
                        100,
                        100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if x1 < cx < x2 and y2 < cy < y3:
                    master.mav.manual_control_send(
                        master.target_system,
                        -100,
                        -100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                if x2 < cx < x3 and y2 < cy < y3:
                    master.mav.manual_control_send(
                        master.target_system,
                        -100,
                        100,
                        0,
                        0,
                        0)
                    time.sleep(0.5)

                master.mav.manual_control_send(
                    master.target_system,
                    0,
                    0,
                    0,
                    0,
                    0)
                time.sleep(15)
                master.mav.manual_control_send(
                    master.target_system,
                    -600,
                    0,
                    750,
                    0,
                    0)
                time.sleep(15)
                master.arducopter_disarm()
                master.motors_disarmed_wait()
                break

        if x3 < cx < x4 and y1 < cy < y3:  # orat sag
            master.mav.manual_control_send(
                master.target_system,
                0,
                200,
                400,
                0,
                0)
            time.sleep(za)

        if x < cx < x1 and y3 < cy < y4:  # asga sol
            master.mav.manual_control_send(
                master.target_system,
                -200,
                -200,
                400,
                0,
                0)
            time.sleep(za)
        if x1 < cx < x3 and y3 < cy < y4:  # asga orta
            master.mav.manual_control_send(
                master.target_system,
                -200,
                0,
                400,
                0,
                0)
            time.sleep(za)

        if x3 < cx < x4 and y3 < cy < y4:  # asga sag
            master.mav.manual_control_send(
                master.target_system,
                -200,
                200,
                400,
                0,
                0)
            time.sleep(za)

        if cx and cy is None:
            master.mav.manual_control_send(
                master.target_system,
                350,
                0,
                330,
                0,
                0)
            time.sleep(za)

    result.write(frem)

    if cv2.waitKey(1) == 27:
        break


def test():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)


vid.release()
cv2.destroyAllWindows()
result.release()
