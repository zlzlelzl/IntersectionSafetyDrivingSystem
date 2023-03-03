# 인지 : 영상검출 차선인식의 정확도 분석

- 목적 : 영상검출 차선인식과 딥러닝 방법의 차선인식 간의 정확도, 소요시간을 분석하여 더 적합한 차선인식 방법의 선정

- 2023. 3. 3일 기준 산출물 : 다양한 운행 환경에 따른 Open-CV기반 차선검출의 정확도를 비교하였습니다.

- 차선검출의 Ground Truth 데이터에 대한 검출 그래프는 아래와 같습니다.
![image](https://user-images.githubusercontent.com/57744586/222616129-f7282f34-1259-4409-8683-d14e331d16cc.png)

- 아래는 맑은 날씨 기준 낮, 박명, 밤, 아침에 대한 차선 검출 그래프입니다.
![image](https://user-images.githubusercontent.com/57744586/222624694-f4d5efd0-d4e6-439e-84b9-8345fe8b0489.png)
![image](https://user-images.githubusercontent.com/57744586/222624812-ae720d50-0ef7-4a02-ad42-a4893c5cf679.png)
![image](https://user-images.githubusercontent.com/57744586/222624898-352c1b13-8357-49bf-ab0b-5932c02103f0.png)

- 결론 : 조도에 따른 인식 결과의 편차가 매우 크게 나왔습니다.  
