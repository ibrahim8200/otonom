import cv2  # opencv kütüphanesi dahil etme
import numpy as np  # Numpy kütüphanesi dahil etme
import sys
import time
import pymavlink.mavlink

from numpy.core.numeric import tensordot

from pymavlink import mavutil

vid = cv2.VideoCapture(0)  # Kamera aktif hale gelir
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


master.arducopter_arm()
master.motors_armed_wait()

frame_width = int(vid.get(3))
frame_height = int(vid.get(4))

size = (frame_width, frame_height)

result = cv2.VideoWriter('filename.avi',
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

while 1:  
    time.sleep(0.2)
    ret, frame = vid.read()                                # Frameleri okunmasi

    frame = cv2.resize(frame, (640, 480))
    gray_img=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    img	= cv2.medianBlur(gray_img,	5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
     
    #center
     
    circles	= cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,120,param1=100,param2=30,minRadius=0,maxRadius=0)
  
     

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),6)
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
            print("x Kordinatı : " + str(i[0])) 
            print("Y Kordinatı : " + str(i[1])) 
            cx = i[0]
            cy = i[1]
            break

    cv2.imshow("Kirmizi Bulunmus Hali", frame)   


    if x < cx < x1 and y < cy < y1:  # ust sol
        master.mav.manual_control_send(
            master.target_system,
            100,
            -200,
            500,
            0,
            0)
        time.sleep(za)
    if x1 < cx < x3 and y < cy < y1:  # ust orta
        master.mav.manual_control_send(
            master.target_system,
            200,
            0,
            500,
            0,
            0)
        time.sleep(za)
    if x3 < cx < x4 and y < cy < y1:  # ust sag
        master.mav.manual_control_send(
            master.target_system,
            200,
            200,
            500,
            0,
            0)
        time.sleep(za)
    if x < cx < x1 and y1 < cy < y3:  # orat sol
        master.mav.manual_control_send(
            master.target_system,
            200,
            -200,
            350,
            0,
            0)
        time.sleep(za)
    if x1 < cx < x3 and y1 < cy < y3:  # orat orta
        master.mav.manual_control_send(
            master.target_system,
            350,
            0,
            500,
            0,
            0)
        time.sleep(0.3)

    if x3 < cx < x4 and y1 < cy < y3:  # orat sag
        master.mav.manual_control_send(
            master.target_system,
            0,
            200,
            500,
            0,
            0)
        time.sleep(za)
    if x < cx < x1 and y3 < cy < y4:  # asga sol
        master.mav.manual_control_send(
            master.target_system,
            200,
            -200,
            500,
            0,
            0)
        time.sleep(za)
    if x1 < cx < x3 and y3 < cy < y4:  # asga orta
        master.mav.manual_control_send(
            master.target_system,
            200,
            0,
            500,
            0,
            0)
        time.sleep(za)
    if x3 < cx < x4 and y3 < cy < y4:  # asga sag
        master.mav.manual_control_send(
            master.target_system,
            200,
            200,
            500,
            0,
            0)
        time.sleep(za)
    if cx and cy is None:
        master.mav.manual_control_send(
            master.target_system,
            100,
            0,
            500,
            0,
            0)
        time.sleep(za)

    result.write(frame)

    cv2.imshow("Frame",frame)              
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
