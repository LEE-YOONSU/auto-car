#!/usr/bin/env python3
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx, ry = [], [] 
    # rx: 경로 시작점의 x좌표와 경로 끝점의 x좌표를 저장하기 위한 빈 리스트
    # ry: 경로 시작점의 y좌표와 경로 끝점의 y좌표를 저장하기 위한 빈 리스트
path_points = []
    # 생성된 경로 상의 모든 점들을 저장할 빈 리스트
planning_done = False
    # 경로 계획이 완료되었는지를 나타내는 bool 변수 
ind = 0
    # 계획된 경로 상의 현재 인덱스를 나타내는 정수 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

pygame.init()
    # pygame 라이브러리를 초기화하고 pygame 기능을 사용하도록 하였다.
screen = pygame.display.set_mode((1280,720))
    # AR태그의 위치의 X좌표가 1142인것으로 보아 screen의 크기를 1280x720 으로 설정하였다.
    # 게임 그래픽을 표시하는데 사용할 창 생성
clock = pygame.time.Clock()
    # 게임 루프에서 시간을 추적하기 위한 clock 객체를 생성한다.

kf = 0.1  # look forward gain
    # 전방시야 이득을 나타내는 변수
    # 차량의 목표지점을 결정하는데 영향을 주는 변수값

dt = 0.1  # time thick
    # 시간간격
    # 경로 추적 알고리즘에서 각 단계마다 경로상의 얼마나 멀리 있는 지점을 따라갈지 결정하는데 사용한다.

Ld = 80  # look ahead distance
    # 전방을 바라보는 거리를 나타내는 변수
    # 차량이 얼마나 멀리 앞을 바라볼지 결정하는 데 영향을 주는 중요한 변수
    # 값이 커지면 경로를 잘 따라가지만 세부적인 경로추적이 힘들수 있다.
    # 반면에 값이 작아지면 세부적인 경로추적이 가능하지만 방향이 갑자기 틀어질수 있기 때문에 경로를 따라가는게 힘들 수 있다.
    # 세부적인 파라미터 조종이 필요한 값 

WB = 8.4  # wheel base
    # 픽셀단위의 바퀴 베이스를 나타내는 변수
    # 차량의 앞뒤 바퀴 사이의 거리를 나타낸다.
    # pure pursuit 함수에서 delta를 구하는데 사용한다.

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행한다.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

# 1. 베지어 곡선
#===================================================== 
# 경로를 생성하기 위해 베지어 곡선을 구현
# start_x, y 차량의 시작좌표
# end_x, y 목표지점
# controlx, y 제어점
#=====================================================

#=====================================================
# delta_t, 각 점 간의 거리에 따른 t의 증가량을 나타낸다. 
# x / dist, x값을 조절하여 점의 수를 조절할 수 있다.
# 점의 수가 많을수록 더 정밀하게 경로를 추종할수 있지만
# 복잡도가 올라가기 때문에 값을 가장 적절하다고
# 생각되는 5로 결정한다.
#=====================================================
def draw_bezier_curve(start_x, start_y, end_x, end_y, control_x, control_y):
    dist = math.sqrt((end_x - start_x) ** 2 + (end_y - start_y) ** 2)
    delta_t = 5 / dist
    points = []

    #2. 베지어 곡선, 곡률 제어
    #=========================================================
    # 시작점과 제어점 끝점에 가중치를 곱하여 곡선의 형태를 조절한다.
    # 점의 수를 결정하는 것이 아닌 점의 좌표를 결정하는 코드
    # 시작점과 끝점의 가중치를 조절하는 방법,
    # 제어점의 가중치를 조절하는 방법을 통해서 
    # 곡선의 정도를 조절할 수 있다.
    #=====================================================
    t = 0 
    while t <= 1:

    #=====================================================     
    # t = 0, 시작점을 의미
    # t = 1, 도착점을 의미 
    # while문으 통해 시작점에서 도착점까지 도달할 때 까지 
    # 점의 수로 결정된 좌표의 수 만큼 
    # 좌표마다 가중치를 주고 새로운 좌표를 추가한다.
    #=====================================================
        x = int((1 - t) ** 2 * start_x + 2 * (1 - t) * t * control_x + t ** 2 * end_x)
        y = int((1 - t) ** 2 * start_y + 2 * (1 - t) * t * control_y + t ** 2 * end_y)
        points.append((x, y))
        print(len(points))
        t += delta_t 
    return points

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global planning_done, rx, ry, path_points, screen
    print("Start Planning")

    rx = [sx, P_ENTRY[0]] 
    # 차량의 위치에 구애받지 않고 주차라인 진입 점까지 경로를 만들어야 하기때문에 
    # 차량 현재 위치의 x좌표와 주차라인 진입점의 x좌표를 저장하고 있는 변수
    ry = [sy, P_ENTRY[1]]
    # 차량의 위치에 구애받지 않고 주차라인 진입 점까지 경로를 만들어야 하기때문에 
    # 차량 현재 위치의 y좌표와 주차라인 진입점의 y좌표를 저장하고 있는 변수
    control_x = (P_ENTRY[0] + rx[0]) // 2
    control_y = (P_ENTRY[1] + ry[1]) // 2
    # control x와 y값에 따라 경로 생성이 곡선으로 그려질수 있는지 없는지 결정되기 때문에 
    # 적절하게 파라미터를 수정하며 적절한 값을 찾았다.
    # 경로를 직선으로 그려내는 것은 실제 우리 현실 환경에서 말이 안된다고 생각헀기 때문에 
    # 곡선으로 나타내고자 한다. (핸들의 영향을 받지 않는 상태가 직선이기 때문)
   
    control_x += int(math.cos(syaw) * max_acceleration * dt)
    control_y += int(math.sin(syaw) * max_acceleration * dt)
    # 시작각도와 최대 가속도, 단위 시간을 활용하여 경로를 조정
     
    path_points = draw_bezier_curve(rx[0], ry[0], P_ENTRY[0], P_ENTRY[1], control_x, control_y)
    print(path_points)
    # path_points : draw_bezier_curve 함수를 통해서 생성된 경로의 모든 x,y좌표 값을 저장할 변수
    # print(path_points) : 경로가 잘 생성되었는지 확인
    # print(path_points) : 이후 주차진입부터 마커까지의 경로 생성 이후 주차가 잘 이루어지는 최적의 x값을 뽑아내기 위해 확인
    
    planning_done = True
    # 경로 생성이 완료되었음을 나타내었음
    return [point[0] for point in path_points], [point[1] for point in path_points]
    # 경로 위의 모든 점을 나타내었다 > 점들이 모여서 경로가 생성된다.

#========================================================================================================
# state 클래스 정의
# 자동차의 상태정보를 나타낸다.
#========================================================================================================
class State:    
    def __init__(self,x,y,yaw,velocity):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity
        self.rear_x = self.x - ((WB/2)*math.cos(self.yaw))  
        self.rear_y = self.y + ((WB/2)*math.sin(self.yaw))
        # pure pursuit 알고리즘 자체가 자동차의 후방 좌표(뒷바퀴 좌표)를 기준으로 목표지점을 설정하기 때문에 자동차의 후방좌표를 나타낸다. 
        self.front_x = self.x + ((WB/2)*math.cos(self.yaw))
        self.front_y = self.y + ((WB/2)*math.cos(self.yaw))
        # 자동차 전방좌표(자동차 앞바퀴)

    def update(self,a,delta):
        # 현재 가속도와 자동차의 각도를 받아 자동차의 상태를 업데이트 하는 함수             
        self.x += self.velocity *math.cos(self.yaw)* dt  
        self.y += self.velocity *math.sin(self.yaw)* dt
        self.yaw+= self.velocity/WB * math.tan(delta) * dt
        self.velocity += a * dt
        self.rear_x = self.x - ((WB/2)*math.cos(self.yaw))
        self.rear_y = self.y + ((WB/2)*math.sin(self.yaw))
        self.front_x = self.x + ((WB/2)*math.cos(self.yaw))
        self.front_y = self.y + ((WB/2)*math.cos(self.yaw))

    def calc_distance(self,point_x,point_y):
        # pure pursuit 알고리즘을 적용하기 위해 자동차의 뒷바퀴 좌표로 부터 떨어진 거리값을 구하는 부분
        # pure pursuit 알고리즘: 자동차 뒷바퀴(후방좌표)로 부터 일정 거리 떨어진 값을 목표점으로 설정
        # point_x : 점의 x좌표 
        # point_y : 점의 y좌표 
        dx= self.rear_x - point_x
        dy= self.rear_y - point_y
        return math.hypot(dx,dy)

#=============================================================================================================================
# states 클래스 정의 : state 객체를 관리 하는 역할
# state 객체를 통해 update되는 x,y,yaw,velocity 값을 추출하고 그 값을 저장해주는 역할
#==============================================================================================================================
class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.velocity = []

    def append(self,state):
    # state 객체에서 x,y,yaw,velocity값을 추출하여 states객체에 추가하고 저장하는 함수
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.velocity.append(state.velocity)

#======================================================================================================================
# TargetCourse 클래스 
# 주어진 경로를 따라 목표지점을 탐색하는 역할
#======================================================================================================================
class TargetCourse:
    def __init__(self, cx, cy):
        self.cx=cx
        self.cy=cy
        self.index_old = None
    def searching_target_index(self, state):
            global path_points,planning_done,ind
            if planning_done :
            # planning 이후에 tracking 힘수가 실행
            # planning 함수에 의해서 plannin done == true 로 결정
            # 조건문은 tracking시 경로 생성이후에 이뤄지기 때문에 항상 참
                print("start tracking")
                if self.index_old is None:
                    dx = [state.rear_x - icx for icx in self.cx]
                    dy = [state.rear_y - icy for icy in self.cy]
                    ind = np.argmin(np.hypot(dx,dy))
                    # pure pursuit 알고리즘을 위해 자동차 후방좌표로 부터 목표지점들간의 거리 계산
                    # 거리값이 가장 최소가 되는 인덱스를 변수 ind에 저장
                    self.index_old = ind
                    # else 문으로 넘어가기 위해서 index_old 에 ind값 저장
                else:
                    # 현재 가장 가깝다고 선택된 ind값이 자동차 후방좌표로 부터 가장 가까운 점이 맞는지 확인하기 위한 부분   
                    ind = self.index_old 
                    distance_this_index = state.calc_distance(self.cx[ind],self.cy[ind])
                    while True:
                        if (ind+1) < len(self.cx) and (ind+1) < len(self.cy):
                            distance_next_index = state.calc_distance(self.cx[ind+1],self.cy[ind+1])
                        if distance_this_index < distance_next_index:
                            break
                    # distance_this_index >= distance_next_index 크게 되면 가장 가까운 인덱스를 잘못찾은것이기 떄문에 밑에 코드로 이동
                        ind = ind+1 if(ind+1) <len(self.cx) else ind
                    # 새롭게 가장 가까운 점을 찾게 되면 distance_this_index에 다시 가까운 거리값 부여 
                        distance_this_index = distance_next_index
                    self.index_old = ind
                Lf = kf* state.velocity + Ld
                # pure pursuit 알고리즘에서 목표지점을 결정하기 위한 기준이 되는 거리 값 
                # 거리값이 가장 작은 index 값을 찾고 ind에 저장한다.
                while Lf > state.calc_distance(self.cx[ind],self.cy[ind]):
                    if (ind+1) >= len(self.cx):
                        break
                    ind += 1 # 다음 인덱스로 이동
                
                # Lf 값이 현재 차량과 목표 지점 간의 거리보다 작아지게 되면
                # 인덱스 탐색을 중지하고 목표 인덱스 값과 Lf 값을 반환한다.
                return ind, Lf
            
#=======================================================================
# pure pursuit 알고리즘
# state: 상태정보, trajectory: 경로정보, pind : 전 타겟 목표지점의 인덱스
#=======================================================================
def pure_pursuit_steer_control(state,trajectory,pind):
        global path_points
        ind , Lf =  trajectory.searching_target_index(state)
        # 현재 차량의 목표 인덱스와와 전방거리 반환값을 받아와 ind,Lf 에 각각 저장한다.
        cx = [point[0] for point in path_points]
        cy = [point[1] for point in path_points]
       
        if pind >= ind:
            ind = pind
        # 경로를 따라 앞으로 이동해야 하기 떄문에 새롭게 찾은 목표지점이 전 지점 목표지점보다 뒤에 있다면 ind == pind로 설정
        if ind < len(cx):
            tx = cx[ind]
            ty = cy[ind]

        else :
            tx = cx[-1]
            ty = cy[-1]
            ind = len(cx)-1
        # 다음 목표지점의 x좌표 : tx, 다음목표지점의 y좌표: ty
        alpha =math.radians(state.yaw)- math.atan2(state.rear_y-ty, tx-state.rear_x)
        # 오른쪽으로 갈수록 x좌표가 증가하고 아래쪽으로 갈수록 y좌표 증가
        # 좌표평면을 고려하여 각도제어
        # 자동차의 헤딩방향 각도로부터 목표지점과 자동차 뒷바퀴가 이루는 각도를 계산
        # 즉, 자동차가 경로를 따라가기 위해 꺾어주어야 하는 각도가 된다.
        print('alpha : ', math.degrees(alpha))
        delta = math.atan2(2.0*WB*math.sin(alpha)/Lf,1.0)
        # pure pursuit 알고리즘의 계산식에 따라 최종 delta값을 결정해준다.

        return delta, ind

#===========================================================================
# 비례 적분
#===========================================================================
# PI class
# kp : 비례항의 가중치
# ki : 적분항의 가중치
# Iterm : 현재 오차에 ki값을 곱한 값
# Pterm : 현재 오차에 kp값을 곱한 값
# last_error : 이전 오차값으로 현재 오차와 비교하여 변화량을 계산하는데 사용한다.
#============================================================================
class PI:
    def __init__(self, kp=10.0, ki=5.0):
        
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
       
        self.Pterm = self.kp * error
    # p항 계산
        self.Iterm += error * dt
    # I항 계산
        self.last_error = error
        output = self.Pterm + self.ki * self.Iterm
        return output
    # 더 정확한 경로 추종을 위해 출력한 결과값

#===================================================================
# 차량이 추종하는 목표 인덱스가 마지막 인덱스와 같을 때
# 주차를 시행함, 주차 또한 베지어 곡선을 사용하여 주차 경로를 생성할 것이다.
# 주차를 위해 경로의 끝점을 AR의 x,y 좌표로 설정한다.
#===================================================================    
def parking(sx, sy, syaw, max_acceleration, dt):
    global planning_done, rx, ry, path_points, screen
    print("Start Parking")
    
    # 주차를 위해 주차진입 지점부터 AR마커까지의 경로 재성성
    rx = [sx,  AR[0]]
    ry = [sy,  AR[1]]

    control_x = (rx[0] + AR[0]) // 2
    control_y = (ry[1] + AR[1]) // 2

    path_points = draw_bezier_curve(rx[0], ry[0], AR[0], AR[1], control_x, control_y)
    print(path_points)

#=========================================================================    
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global path_points
    global planning_done, rx, ry, path_points

    print("start tracking")

    #======================================================
    # 경로상에 존재하는 좌표를 저장한 변수 path_points를
    # 인덱싱과 반복문을 통해 새로운 형태의 변수로 저장한다.
    # trcaking deafault 선속도를 30으로 지정한다.
    #======================================================
    cx = [point[0] for point in path_points]  # 경로의 x좌표
    cy = [point[1] for point in path_points]  # 경로의 y좌표
    target_speed = 30
    
    #==============================================================
    # 현재 차량의 상태를 저장하는 변수를 생성.
    # path 길이가 len으로 지정되었으니 index로 표현하기 위해 -1을 해줌
    # 경로상의 x,y 좌표를 통해 최적의 목표지점을 탐색하는 변수를 생성.
    # 현재 차량의 상태를 나타내는 변수 state를 기반으로
    # 목표 index를 결정하는 변수를 생성.
    #==============================================================
    state = State(x,y,yaw,velocity)
    lastIndex = len(cx)-1  
    target_course=TargetCourse(cx,cy)
    target_ind,_ = target_course.searching_target_index(state) 

#후진 및 경로 재생성
#===============================================================================
    # 로봇의 시작 위치가 P_ENTRY와 너무 가깝고 
    # P_ENTRY와의 각도 차이가 너무 클 때, 경로를 생성하여 추종하는데 어려움을 겪었다.
    # 때문에 우리 팀은 일정 거리 후진 후 새로운 경로를 생성하는 방식을 채택했다.
#===============================================================================
    distance_to_end = math.sqrt((rx[0] - P_ENTRY[0]) ** 2 + (ry[0] - P_ENTRY[1]) ** 2)
    distance_to_start = math.sqrt((x - P_ENTRY[0]) ** 2 + (y - P_ENTRY[1]) ** 2)
    print("현위치에서 끝점 거리", distance_to_start)
    print("시작점과 끝점 거리:", distance_to_end)
    #로봇의 시작 위치와 경로의 끝 점까지의 거리(static)를 계산 
    #로봇의 실시간 위치와 경로 끝 점까지의 거리(dynamic)를 계산
    
    # 조건문
    #================================================================================
    # distance_to_end 변수의 크기가 특정 수보다 작고, distance_to_end 변수가 0이 아닐 때:
    # distance_to_end 변수가 0일 때를 넣어준 이유? 
    # 로봇이 경로점을 다 추종하고 나면 distance_to_end 값이 0으로 반환되어 
    # parking을 시작하지 않고 후진하는 경우가 발생한다.
    # for i in range() 함수를 사용하여 후진을 반복하는 코드를 수행 --> 후진
    # distance_to_start 변수가 370이 되었다면 새로운 경로점을 생성한다.
    #================================================================================
    if distance_to_end < 190 and distance_to_end != 0 :
        for i in range(30):
            drive(0, -20) 
        
        if distance_to_start > 370:

            print("re planning")
            rx = [x,  P_ENTRY[0]]
            ry = [y,  P_ENTRY[1]]
            control_x = (x + P_ENTRY[0]) // 2
            control_y = (y + P_ENTRY[1]) // 2

            path_points = draw_bezier_curve(rx[0], ry[0], rx[1], ry[1], control_x, control_y)    
            print(path_points)

#============================================================================================
    # 마지막 인덱스보다 목표 인덱스가 작을 때까지 시행되는 부분, 이가 같아지면 주차를 시행할 것이다.
    if lastIndex > target_ind:
        # 업데이트 주기 설정
        clock.tick(1000)
        
        # 비례-적분 제어를 사용하기 위해 변수 선언 
        pi_acc = PI()
        pi_yaw = PI()

        # 비례-적분 제어 사용
        #=====================================================================
        # default 값으로 저장한 속도와 현재 속도의 차이를 변수로 저장 -> 속도 오차
        # 위에서 선언한 비례-적분 제어기 변수를 사용하여
        # 속도 오차를 기반으로 가속도를 결정 -> acc
        #=====================================================================
        vel_error = target_speed - velocity
        acc = pi_acc.control(vel_error)
        di, target_ind = pure_pursuit_steer_control(state,target_course,target_ind)
        #===========================================================================
        # 현재 차량의 상태와 목표 경로를 바탕으로 
        # 목표 인덱스와 조향각을 결정하는데 이를 새로운 변수로 선언
        # 목표 인덱스 -> target_ind, 조향각 -> di     
        # di, 차량의 목표 각도를 비례-적분 제어기를 통해 새롭게 조향각을 결정 -> deltar.
        # 차량의 현재 상태를 나타내는 state 변수에 새롭게 반영된 
        # 가속도와 조향각을 update.
        # update된 state를 바탕으로 drive, 주행을 시행.
        #===========================================================================
        deltar = pi_yaw.control(di)        
        state.update(acc,deltar)
        drive(math.degrees(deltar),state.velocity)
    
        # 주차 시행 및 주차 종료 조건
        #===========================================================
        # 목표 인덱스가 마지막 인덱스와 같아졌을 때, 주차를 실행한다.
        # 만약 차량의 x값이 1085와 AR의 X좌표 사이에 들어온다면 주차를 완료한다.
        # Parking 함수의 print(path_points) 구문을 이용하여 주차가 완료되는 지점까지의 x좌표를 계속 추출하며 안정적인 파라미터 값을 결정한다.
        # 주차가 안정적으로 완료되는 x좌표를 1085로 결정했다.
        # 주차를 성공했다고 판단하고 주행을 종료한다.
        #===========================================================
    if target_ind == lastIndex:
            parking(P_ENTRY[0],P_ENTRY[1],state.yaw, max_acceleration,dt)
            if 1085 <state.x < AR[0] :
            
                drive(0,0)
