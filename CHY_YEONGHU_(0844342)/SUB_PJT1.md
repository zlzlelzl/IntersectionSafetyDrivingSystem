# 1. 프로그램 설치 및 환경 설정

- 프로그램 동작 환경 및 개발 환경
	- Simulator : Windows 11
	- Dev tool : **Ubuntu 18.04** , **VS Code** on Virtual Box VM env.
	- Language  & Library : Python 3.7, opencv

- ROS와 Simulator 통신 방법
- VM 내 Unbuntu 의 네트워크 주소를 192.168.56.101로 포팅하고, Simulator 의 접속 네트워크 주소를 192.168.56.101로 설정해서 **ROS - Sumulator 간 WebSocket 통신을 할 수 있도록 구성**하였습니다.

![image](https://user-images.githubusercontent.com/57744586/222610613-d7e8cf1a-ce90-48af-a332-d983152bf96b.png)  
roscore 실행 이미지

![image](https://user-images.githubusercontent.com/57744586/222610736-f2e9c36e-b4dd-4147-9dce-ab40afc21bb3.png)  
Ubuntu 네트워크 아이피 설정

![image](https://user-images.githubusercontent.com/57744586/222610916-d81e1031-8635-4a5b-8649-950cbe060931.png)  
스켈레톤 코드 폴더 다운로드
