우분투 터미널에서 아래 명령어 입력

rostopic pub /ctrl_cmd Morai_msgs/CtrlCmd "{longlCmdType: 1, accel: 0.6, brake: 0.0, steering: 0.1, velocity: 0.0, acceleration: 0.0}"

longlCmdType : 제어 방식을 결정하는 인덱스로
- longCmdType == 1이면 Throttle 제어(accel/brake/steering)
- longCmdType == 2이면 Velocity 제어(velocity/steering)
- longCmdType == 3이면 Acceleration 제어(acceleration/steering)

accel : 차량의 엑셀값을 의미하며 0 ~ 1의 범위
brake : 차량의 브레이크값을 의미하며 0 ~ 1의 범위
steering : 차량의 바퀴 각도(rad)
velocity : longlCmdType이 2일 경우 사용(km/h)
acceleration : longlCmdType이 3일 경우 사용(m/s^2)