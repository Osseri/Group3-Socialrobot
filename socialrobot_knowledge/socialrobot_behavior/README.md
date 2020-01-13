# socialrobot_behavior

## Dependencies

- ROS Kinetic/Melodic
- [vision_msgs](https://github.com/Kukanani/vision_msgs)
- [moveit](https://github.com/ros-planning/moveit)
- socialrobot_motion
- socialrobot_hardware
- pyassimp > 4.1.3

## Install

1. Install the ROS. [Instructions for Ubuntu 16.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
   
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
   
3. Install vision_msgs ROS package
   ```
   sudo apt-get install ros-kinetic-vision-msgs
   ```   
4. Install moveit ROS package
   ```
   sudo apt-get install ros-kinetic-moveit
   ```

5. make
   ```
   catkin_make
   ```

6. If pyassimp error occurred when launch the package, upgrade pyassimp version upper than 4.1.3


## Guide for adding Behaviors
1. 행위에 대한 motion planner들은 script/behaviors 안에 python 모듈로 작성해야함.
   
    예) 물체 접근 행위`approach.py`, 로봇 그리퍼 open/close 행위`openclose.py`, 로봇 팔 이동 행위`movearm.py`

2. 행위 모듈 클래스들은 script/behaviros 안의 behavior.py를 import하여 BehaviorBase를 상속받고 클래스명은 행위이름 뒤에 `Behavior`를 붙임.
   
   예) approach행위:
   ```
   from behavior import BehaviorBase

   class ApproachBehavior(BehaviorBase):
   ```
3. 행위클래스를 정의할때 다음의 함수들이 오버라이딩으로 구현되어야 함.
   ```
   def check_requirements()         #행위에 필요한 정보들(필요 하드웨어 대상, 컨트롤러 등)을 응답
   def prepare_behavior()           #행위 시작 전 컨트롤러 셋업
   def run_behavior()               #행위 시작 명령
   def finish_behavior()            #행위 종료 응답
   def get_motion()                 #행위 모션 결과 응답
   ```

## Behavior Demo
V-REP 시뮬레이션을 통한 검증은 다음과 같음.
1. socialrobot 관련 패키지들을 설치. Sociaorobot gitlab 참고.
2. roscore 실행 후, vrep 실행
3. 필요 ROS 패키지들을 launch
   ```
   roslaunch socialrobot_state demo_init.lauunch
   ```

4. 행위에 필요한 파라미터들을 `GetMotion.srv` 서비스를 통해 request
5. responce로 받은 motion trajectory를 `SetBehavior.srv` 서비스를 통해 request하여 vrep 시뮬레이션 내 결과 확인

`example/`의 예제 코드 참고.