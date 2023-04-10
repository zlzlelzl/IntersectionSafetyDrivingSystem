# 포팅 메뉴얼

## 실행 과정

1. Git Clone
    - 아래 명령어를 차례로 실행하여 프로젝트 폴더를 받습니다.
        
        `git init`
        
        `git config core.sparseCheckout true`
        
        `git remote add -f origin [https://lab.ssafy.com/s08-mobility-autodriving-sub2/S08P22A709.git](https://lab.ssafy.com/s08-mobility-autodriving-sub2/S08P22A709.git)`
        
        `echo "project_isds" >> .git/info/sparse-checkout`
        
        `git pull origin master`
        
2. 개발 환경 설정
    - 개발 환경 설치
        1. Virtual Box 다운로드 후 설치([https://www.virtualbox.org/wiki/Downloads](https://www.virtualbox.org/wiki/Downloads))
        2. **Ubuntu 18.04**  다운로드 후 설치
            
            [https://releases.ubuntu.com/18.04/](https://releases.ubuntu.com/18.04/)
            
        3. Virtual Box 에 Ubuntu 설치
            - [ ]  디스크 이미지는 2번에서 받은 Ubuntu 18.04 iso 파일 선택
            - [ ]  가상 머신의 메모리는 4GB이상으로 설정
            - [ ]  가상 머신의 하드 디스크는 `새로 만들기` 선택
            - [ ]  가상 머신 하드 디스크 종류는 `VDI` 로 설정
            - [ ]  가상 머신 하드 디스크 할당 방식은 `동적 할당` 으로 설정
            - [ ]  가상 머신 하드 디스크 크기는 30GB 이상으로 설정
            
        4. 
        
        - /
3. 빌드
    - 작업 공간 생성
        
        `$ mkdir -p ~/catkin_ws/src`
        
        `$ cd ~/catkin_ws/src`
        
        `$ catkin_init_workspace`
        
    
    - ~/catkin_ws/src 내부에서 git clone
        
        `$ cd ~/catkin_ws/src`
        
        `$ git clone [https://lab.ssafy.com/s08-mobility-autodriving-sub2/S08P22A709.git](https://lab.ssafy.com/s08-mobility-autodriving-sub2/S08P22A709.git)`
        
    - catkin_make
        
        `$ cd ~/catkin_ws`
        
        `$ catkin_make`
        
4. 실행
    
    `$ roslaunch rosbridge_server rosbridge_websocket`
    
    `$ [r](https://lab.ssafy.com/s08-mobility-autodriving-sub2/S08P22A709.git)oslaunch project_isds kcity_driving.launch`
