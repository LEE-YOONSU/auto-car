#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import rospy, rospkg, time
import numpy as np
import cv2, math
import rospy, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import time

#=============================================
# 터미널에서 Ctrl-c 키입력이로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, image):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge()
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480   # 카메라 이미지 가로x세로 크기

# #=============================================
# # 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# # 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# # 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
# #=============================================


def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

    

def bird_eye_view(image): 
    #차선을 위에서 보는것처럼 이미지 화면을 바꿔주는 함수
    #bird_eye_view를 이용해 주행경로를 더 자세히 파악하고, 객체를 저 정확하게 탐지하고 추적할 수 있기 때문에 bird_eye_view를 사용
    img_size = (image.shape[1],image.shape[0]) #이미지 사이즈를 가로, 세로 순서대로 지정
    src = np.float32(np.array([[230,290],[20,400],[420,290],[620,400]])) 
    #차선을 일직선으로 보이게 하기위해 차선이 지나는 적절한 좌표(x1, y1,x2,y2)를 하나하나 직접 찾아 src라는 변수에 최적의 값을 저장한다.
    dst = np.float32([[0,0],[0,480],[640,0],[640,480]]) 
    # 이미지의 넓이와 높이 값을 지정해준다.
    

    matrix = cv2.getPerspectiveTransform(src,dst) #원근법 변환: 직선의 성질만 유지되고, 선의 평행성은 유지가 되지않는 변환 
    # 이 변환을 이용하면 차선이 일직선에 가까운 모습으로 변환됩니다.
    minv = cv2.getPerspectiveTransform(dst,src)

    bird = cv2.warpPerspective(image,matrix,img_size) 
    #warpPerspective 함수에 변환 행렬값을 적용하여 최종결과 이미지를 bird라는 함수에 저장합니다.
    cv2.imshow('bird', bird) #bird_eye_view 나타내기
    cv2.waitKey(1)

    return bird, minv #변환 행렬값 리턴하기

def color_filter(image): 
    # 변환행렬을 수행한 이미지를 바탕으로 차선만 인식하는 마스킹 기능
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS) #주어진 이미지를 BGR색상 공간에서 HLS(Hue, Luminance, Saturation)색상 공간으로 변환

    lower = np.array([0, 200, 0]) #흰색 범위를 정의하는 임계값을 설정합니다.
    #H값은 0으로 설정되어 있어 모든 색상을 허용
    #L값은 200으로 설정되어 있으므로, 200 이상의 밝기를 가진 픽셀만 흰색으로 간주
    #S값은 0으로 설정되어 있어 모든 채도의 픽셀을 흰색으로 간주. 따라서 채도가 낮은 픽셀도 흰색으로 선택됨
    upper = np.array([255, 255, 255])
    # 이 값은 흰색의 최대범위를 의미, 따라서 [255,255,255] 범위 내에 있는 모든 픽셀은 흰색으로 간주
    #요약해서 상한, 하한 임계값 범위내에 있는 픽셀은 흰색으로 선택된다.

    white_mask = cv2.inRange(hls, lower, upper)
    #hls 이미지에서 흰색에 해당하는 부분을 찾아 이진 마스크 이미지로 생성
    # 흰색에 해당하는 픽셀은 흰색으로 그 외의 픽셀은 검은 색으로 표시
    masked = cv2.bitwise_and(image, image, mask=white_mask)
    #image 와 whitemask를 이용해 픽셀별로 bitwise AND연산을 수행
    # 그 결과 흰색 마스크가 있는 영역만 남기고 나머지 영역은 제거된 이미지 masked가 생성
    cv2.imshow('masked', masked)
    cv2.waitKey(1)

    return masked


def roi(image):
    #차선 중앙의 마름모같은 그림들이 차선인식에 영향을 받지 않게 하기 위해 지정된 영역 외 부분은 인식하지않는 함수
    x = int(image.shape[1])
    y = int(image.shape[0])

    
    _shape = np.array(
        [[int(0), int(y)], [int(0), int(0.1*y)], [int(x), int(0.1*y)], [int(x), int(y)], [int(0.7*x), int(y)], [int(0.7*x), int(0.5*y)],[int(0.3*x), int(0.5*y)], [int(0.3*x), int(y)], [int(0), int(y)]])
    #차선 중앙에 있는 마름모 또는 다른 그림들이 차선으로 인식되지 않게 하기 위해 
    # 차선의 주위만 인식할 수 있도록 마스킹 되는 다각형을 만들어준다.
    #마름모가 보이지 않도록 다각형의 좌표를 하나하나 바꿔가며 차선은 보이면서 중간에 마름모는 보이지않는 좌표를 찾는다.
    #슬라이드 윈도우가 0~240에서 움직여야하고 히스토그램으로 인해 차선안의 240~480의 영역의 차선만 인식하기 때문에 y:240~480까지의 범위만 검정색으로 마스킹한다.
    mask = np.zeros_like(image)
    #mask라는 이름의 동일한 크기를 가진 검은색 이미지 생성
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    
    cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
    #mask 이미지에 _shape로 정의된 다각형을 그린다.
    #이를 통해 다각형 영역이 흰색으로 채워짐
    masked_image = cv2.bitwise_and(image, mask)
    #bitwise_and() 함수를 사용하여 'image' 'mask'를 비트 연산하여 원본 이미지에서 마스킹된 영역만 추출
    cv2.imshow('maskedimage', masked_image)
    cv2.waitKey(1)
    return masked_image   

def plothistogram(image):
    # 히스토그램에서 왼쪽 차선과 오른쪽 차선의 기준점을 찾고 그 기준점을 기준으로 
    # 차량의 조향각을 찾기 위해 이 함수를 사용  
    histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
    # 차선을 인식하는 부분을 이미지의 절반 아래로 지정하고 각 열의 픽셀값을 다 더한다.
    midpoint = np.int(histogram.shape[0]/2)
    # 히스토그램의 중앙 인덱스 계산
    leftbase = np.argmax(histogram[:midpoint])
    # 히스토그램 변수에서 가장 큰 값의 인덱스를 왼쪽 차선 지점으로 설정한다.
    rightbase = np.argmax(histogram[midpoint:]) + midpoint
    # 히스토그램 변수에서 가장 큰 값의 인덱스를 오른쪽 차선 지점으로 설정한다.
    return leftbase, rightbase


def slide_window_search(binary_warped, left_current, right_current):
    # 슬라이드 윈도우: 고정 사이즈의 윈도우가 이동하면서 윈도우 내에 있는 데이터를 이용하는 알고리즘
    # HoughlinesP를 사용하지 않은 이유로는 slide window가 houghline함수보다 곡선에 대한 차선인식이 정확하기 때문
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))
    # 이진화된 이미지를 컬러 이미지로 변환한 것, 슬라이딩 윈도우를 그리기 위해 사용
    nwindows = 8
    #그릴 슬라이드 윈도우의 개수를 설정함
    window_height = np.int(binary_warped.shape[0] / nwindows)
    #윈도우 높이 계산
    nonzero = binary_warped.nonzero()  # 선이 있는 부분의 인덱스만 저장
    nonzero_y = np.array(nonzero[0])  # 선이 있는 부분 y의 인덱스 값
    nonzero_x = np.array(nonzero[1])  # 선이 있는 부분 x의 인덱스 값
    margin = 50
    minpix = 10
   
    left_lane = []
    right_lane = []
    color = [0, 255, 0]
    thickness = 2

    for w in range(nwindows):
        win_y_low = binary_warped.shape[0] - (w + 1) * window_height  # window 윗부분
        win_y_high = binary_warped.shape[0] - w * window_height  # window 아랫 부분
        win_xleft_low = left_current - margin  # 왼쪽 window 왼쪽 위
        win_xleft_high = left_current + margin  # 왼쪽 window 오른쪽 아래
        win_xright_low = right_current - margin  # 오른쪽 window 왼쪽 위
        win_xright_high = right_current + margin  # 오른쪽 window 오른쪽 아래

        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), color, thickness)
        cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), color, thickness)
        # cv2.rectangle() 함수를 사용해 out_img에 윈도우를 시각화하기 위해 사각형을 그림
        good_left = abs((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
        good_right = abs((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
        #good_left와 good_right는 윈도우 내에서 유효한 왼쪽 차선과 오른쪽 차선의 픽셀 인덱스를 계산한 배열

        left_lane.append(good_left)
        right_lane.append(good_right)

        if len(good_left) > minpix:
            left_current = np.int(np.mean(nonzero_x[good_left]))
        if len(good_right) > minpix:
            right_current = np.int(np.mean(nonzero_x[good_right]))
        #각 윈도우의 흰색 픽셀 인덱스를 left_lane, right_lane리스트에 추가
        cv2.imshow("oo", out_img)

       

    left_lane = np.concatenate(left_lane)
    # np.concatenate() -> array를 1차원으로 합침
    right_lane = np.concatenate(right_lane)
    #np.concatenate()를 사용하여 left_lane과 right_lane 리스트를 연결하여 좌측 및 우측 차선의 인덱스를 단일 배열로 얻습니다
    print('left lane:',left_lane)
    print('right lane:',right_lane)

    leftx = nonzero_x[left_lane]
    lefty = nonzero_y[left_lane]
    rightx = nonzero_x[right_lane]
    righty = nonzero_y[right_lane]
    # 식별된 차선 픽셀의 x 및 y 좌표를 이전 단계에서 얻은 nonzero_x 및 nonzero_y 배열에서 추출
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    #np.polyfit() 함수를 사용하여 좌측 및 우측 차선 픽셀에 2차 다항식 곡선을 맞추기 위한 다항식 계수를 계산합니다. left_fit와 right_fit 변수에 계수를 저장
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    #ploty 배열을 생성하여 곡선 포인트의 y좌표를 나타냄
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    #다항식 계수와 ploty 배열을 사용하여 좌측 및 우측 차선의 곡선 포인트의 x 좌표를 계산
    ltx = np.trunc(left_fitx)  # np.trunc() -> 소수점 부분을 버림
    rtx = np.trunc(right_fitx)

    print('ltx:',ltx)
    
    print('rtx:',rtx)
    out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
    out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]
    # out_img를 업데이트하여 좌측 차선은 파란색, 우측 차선은 빨간색으로 식별된 차선 픽셀을 표시
    cv2.imshow('out_img:',out_img)
    cv2.waitKey(1)

    ret = {'left_fitx' : ltx, 'right_fitx': rtx, 'ploty': ploty}
    #결과로는 계산된 좌측 차선 x 좌표(ltx), 우측 차선 x 좌표(rtx), 그리고 ploty 배열이 포함된 사전(ret)
    return ltx, ret

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(Angle, Speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
#=============================================
def start():
    global motor
    global image
    global Width, Height
    rospy.init_node('img_sub')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print ("----------  ----------")
    time.sleep(3)
    while not rospy.is_shutdown():
        while not image.size == (WIDTH*HEIGHT*3):
            continue
        img = image.copy()
        speed = 6 
        
        cv2.imshow('image', img) 
        cv2.waitKey(1)
        wrapped_img, minverse=bird_eye_view(image)
        w_f_img = color_filter(wrapped_img)
        w_f_rimg = roi(w_f_img)
        
        _gray = cv2.cvtColor(w_f_rimg, cv2.COLOR_BGR2GRAY)
        thresh = cv2.threshold(_gray, 200, 255, cv2.THRESH_BINARY)
        #이진화를 하기 위해 하나의 채널을 가진 grayscale 이미지로 바꿔주거야 함

        cv2.imshow('thresh',thresh)
        leftbase, rightbase = plothistogram(thresh)
        draw_info = slide_window_search(thresh,leftbase,rightbase)
        gradient = round((draw_info[0] - 80) / 120, 1)

        if gradient > 1.0:
            speed = 3
            drive(gradient+0.5, speed)
        else:
            drive(gradient, speed)
        
if __name__ == '__main__':
    start()
