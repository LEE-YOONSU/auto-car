#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

ultrasonicData = [0.0, 0.0, 0.0, 0.0, 0.0]

def callback(msg): 
    global ultrasonicData
    ultrasonicData = msg.data  

rospy.init_node('driver')
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

xycar_msg = xycar_motor()

while not rospy.is_shutdown():
#=========================================================
# 초음파 센서를 통해 회피 방향 결정 및 주행 속도 결정
# default 값으로 선속 45, 각속 0으로 부여.
#=========================================================

    speed = 45
    angle = 0

# 코드의 전반적인 설명
#=========================================================
# 자동차를 제어하는데 1번, 2번, 3번 초음파 센서
# 만 있어도 제어가 가능하다고 판단하여 
# 3개의 초음파 센서에 대한 조건문을 통해 제어를 구현한다.
# 1번과 3번 초음파를 통해 회피 방향을 결정하고
# 2번 초음파는 차량의 속도를 결정할 것이다.
#=========================================================


# 1. 우회전
#=====================================================================
# 2번 초음파(중앙)와 가까운 왼쪽에 위치한 3번 초음파 센서의 값이
# 2번 초음파(중앙)와 가까운 오른쪽에 위치한 1번 초음파 센서값보다 더 클 때
# angle 35 -> 우회전 
#=====================================================================
    if ultrasonicData[1] < ultrasonicData[3]:
        angle = 35
        print(speed, ultrasonicData[2])

# 2. 좌회전
#=====================================================================
# 2번 초음파(중앙)와 가까운 오른쪽에 위치한 1번 초음파 센서의 값이
# 2번 초음파(중앙)와 가까운 왼쪽에 위치한 3번 초음파 센서값보다 더 클 때
# angle 35 -> 좌회전
#=====================================================================
    if ultrasonicData[1] > ultrasonicData[3]:
        angle = -35
        print(speed, ultrasonicData[2])

# 3. 선속 제어
#====================================================
# 2번 초음파에서 측정되는 거리에 따라 선속에 변화를 준다.
# 측정되는 거리가 200보다 작으면 선속 45
# 측정되는 거리가 200보다 크면 선속 50
# 측정되는 거리가 350보다 크면 선속 90
# 측정되는 거리가 500보다 크면 선속 110
# 거리가 더 높게 측정될수록 선속을 더 빠르게 할당하여
# 빠른 주행을 구현하고자 한다.
#====================================================
    if ultrasonicData[2] > 200:
        speed = 50
        print(speed, ultrasonicData[2])
  
    if ultrasonicData[2] < 200:
        speed = 45
        print(speed, ultrasonicData[2])

    if ultrasonicData[2] > 350:
        speed = 90
        print(speed, ultrasonicData[2])

    if ultrasonicData[2] > 500:
        speed = 110
        print(speed, ultrasonicData[2])
    
    xycar_msg.angle = angle
    xycar_msg.speed = speed
    motor_pub.publish(xycar_msg)