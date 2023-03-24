[ 삼성 청년 SW 아카데미 (SSAFY) 8기 - 2학기 특화프로젝트 ]

# ISDS

## I. 서비스 소개

### 개요

- **I**ntersection **S**afety **D**rive **S**ystem
(교차로 안전 운전 시스템)
- **자율주행 환경**에서 교차로 내 **우회전** 시 **고도화된 상황대처 시스템**

### 타겟

- **우회전 시 발생하는 돌발 사고를 예방**하고 싶은 자율주행 차량 운전자
- 교차로 우회전 시 돌발상황 대처 능력이 상대적으로 부족한 **초보운전자**

## II. 기획 배경

### 배경

![https://user-images.githubusercontent.com/57744586/227391880-43309a7d-cd91-401f-8073-bf3f6c933575.png](https://user-images.githubusercontent.com/57744586/227391880-43309a7d-cd91-401f-8073-bf3f6c933575.png)

- **최근 수년간 우회전 교통사고 보행 사상자 수치는 유지 중**
- **앞으로 급격하게 성장 예정인 자율주행 차량 시장**
- **자율주행 차량의 우회전 상황에서의 고도화된 상황 대처 시스템이 필요**

### 목적

- **자율주행 시장의 성장성**, **줄어들지 않는 교차로 우회전 사고율**을 해결하기 위해 **ISDS** 시스템을 개발

### 의의

- 우회전 상황에서의 자율주행 차량의 상황 대처 능력 향상으로 사고율 감소

## III. 주요 기능

- 기본적인 자율주행 SW에 **ISDS** SW모듈이 탑재된 형태
- **ISDS**
    - 우회전 직전 정지선에서의 일시정지
    - 우회전 시 신호등 확인 후 진입
    - 우회전 시 보행자 감지 상황에서 일시정지
    - 우회전 시 좌측에 직진 차량 대처
    - 보행자의 급작스런 돌발행동 상황 대처 (갑자기 뛰어오는 어린이, 자전거 역주행, 무단횡단 등)

## IV. 기술

- **공통**
    - 개발언어 : Python 3.7
    - Framework : ROS melodic 1.0
    - Simulator : Morai Simulator
    - 개발 환경 : Ubuntu
- **인지**
    - LiDAR, Camera, GPS, IMU
    - open CV
    - Lane detection based Canny Edge & RANSAC Algorithm
    - SFA3D (Super Fast Accurate 3D Object Detection based on 3D LiDAR Point Cloud )
    - Yolo-X
- **판단**
- **제어**
    - Pure-Pursuit Algorithm
    - PID control
    - ACC (Adaptive Cruise Control)

## VI. 협업 툴 및 협업 환경

- **Notion**
    - 기획 및 회의록 작성
    - 산출물 기록 및 공유
- **JIRA**
    - 매주 목표량 설정하여 sprint 진행
    - 업무 할당량 정하여 Story Point 설정, In Progress > Done 순으로 작성
- **GitLab**
    - 코드 버전 관리
    - 이슈 발행 및 관리

## VII. 역할 분담

- **인지**
    - 이준혁 :
        - SFA3D 모델을 이용한 3d object detection
        - Motion Forecasting
    - 차영후
        - 차선 인식 알고리즘 설계 및 개선
        - Yolo-X model 신호등 Dataset 학습 및 적용
        - 인지 파트 SW설계
        - 개발 및 최적화
- **판단**
    - 김호준 :
        - Adaptive Cruise Control 전방 NPC 차량 인식 개선
    - 장지웅 :
        - 환경 설정
        - 정지선 판단
- **제어**
    - 김승기 :
        - 종방향 제어
    - 조은비 :
        - 횡방향 제어

## VIII. 프로젝트 결과물

※ 2023. 03. 24(4주차) 기준

- **인지**
    - **차선 인식 알고리즘을 개선**
        
        (기존 white-yellow color 검출 기반 ⇒ Canny Edge 기반)
        
    - **다양한 환경(악천후, 박명, 일몰, 야간 등)에서의 차선 검출 정확도 향상**
        
        (개선 전 대비 57% 향상 - Ground truth 데이터 기준으로 정확도 산출)
        
        ![Untitled](https://user-images.githubusercontent.com/57744586/227422714-305ae9b9-bfec-4785-acae-e0fc62f344bd.png)
        
    
    - **ISDS개발**
        - ISDS 인지 파트 플로우 (설계)
        
        ![Untitled 1](https://user-images.githubusercontent.com/57744586/227422751-13751cf4-4ded-4c5e-ba56-af9ed3c67db4.png)
        
- **판단**

![stoplane_not_in_route](https://user-images.githubusercontent.com/57744586/227422888-a81cf6ba-c0e2-4668-9c44-710ffed1e685.png)

![stoplane_in_route](https://user-images.githubusercontent.com/57744586/227422916-a4e65b4d-c9b5-47c4-bda3-61d9e5098ced.png)

![crosswalk](https://user-images.githubusercontent.com/57744586/227422936-8fe29cbd-37bd-49f4-8a87-2bf87c4dc14f.png)

![Simulator-2023-03-24-10-53-13-_online-video-cutter.com_-_1_.gif](https://s3.us-west-2.amazonaws.com/secure.notion-static.com/fce67bab-93ab-46ae-ad95-341fc8e9424e/Simulator-2023-03-24-10-53-13-_online-video-cutter.com_-_1_.gif?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Content-Sha256=UNSIGNED-PAYLOAD&X-Amz-Credential=AKIAT73L2G45EIPT3X45%2F20230324%2Fus-west-2%2Fs3%2Faws4_request&X-Amz-Date=20230324T042011Z&X-Amz-Expires=86400&X-Amz-Signature=f3c5c3e55d5dd427a039667db1614f8ce431ffd5b2160059ac65e115ba7c081d&X-Amz-SignedHeaders=host&response-content-disposition=filename%3D%22Simulator-2023-03-24-10-53-13-_online-video-cutter.com_-_1_.gif%22&x-id=GetObject)

- **제어**
    - 종방향 제어
        - 곡률에 따른 속도 제어 (가속, 감속 제어)
            - 좌/우회전 시 최대 속도 100km/h 부터 감속 비교
            
            ![종방향제어before.gif](https://s3.us-west-2.amazonaws.com/secure.notion-static.com/4bafa483-0810-4b81-a835-52b6344573ee/%EC%A2%85%EB%B0%A9%ED%96%A5%EC%A0%9C%EC%96%B4before.gif?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Content-Sha256=UNSIGNED-PAYLOAD&X-Amz-Credential=AKIAT73L2G45EIPT3X45%2F20230324%2Fus-west-2%2Fs3%2Faws4_request&X-Amz-Date=20230324T042132Z&X-Amz-Expires=86400&X-Amz-Signature=d4b1385a7bcb8c175beae720a131027fde9ffa5528616a5153d10dcfd13c713f&X-Amz-SignedHeaders=host&response-content-disposition=filename%3D%22%25EC%25A2%2585%25EB%25B0%25A9%25ED%2596%25A5%25EC%25A0%259C%25EC%2596%25B4before.gif%22&x-id=GetObject)
            
            
            ![종방향제어after.gif](https://s3.us-west-2.amazonaws.com/secure.notion-static.com/cb47f5e1-17cc-45d5-83f2-f8675869c8e8/%EC%A2%85%EB%B0%A9%ED%96%A5%EC%A0%9C%EC%96%B4after.gif?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Content-Sha256=UNSIGNED-PAYLOAD&X-Amz-Credential=AKIAT73L2G45EIPT3X45%2F20230324%2Fus-west-2%2Fs3%2Faws4_request&X-Amz-Date=20230324T042202Z&X-Amz-Expires=86400&X-Amz-Signature=b841a980e04fc1cb61b3429f2249e9d0be660a44a7e2560b4edf46bd2c2ea248&X-Amz-SignedHeaders=host&response-content-disposition=filename%3D%22%25EC%25A2%2585%25EB%25B0%25A9%25ED%2596%25A5%25EC%25A0%259C%25EC%2596%25B4after.gif%22&x-id=GetObject)
            
    - 횡방향 제어

## VIII. 서비스 데모 영상

- **Demo Video**
