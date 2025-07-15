<h1 align="center">ROS2를 활용한 로봇 자동화 공정 시스템 구현🔬 </h1>

<h2 align="center">굿즈 이미지 제작 자동화📝 </h2>



## 개요


빈번한 수정과 반복작업이 많은 디자인 직무(캐릭터, 타투 도안 제작 등)에서 인간의 육체적 노동을 로봇이 대체하여 피로도를 줄이고 제작자의 순수 창작 집중도 증가






## 제작 기간 & 참여 인원


-2025/06/09~2025/06/20  4명






## 사용한 기술 (기술 스택)  


<img src="https://img.shields.io/badge/python-blue?style=for-the-badge&logo=python&logoColor=white">   <img src="https://img.shields.io/badge/ROS2-black?style=for-the-badge&logo=ros&logoColor=#22314E">   <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white">    <img src="https://img.shields.io/badge/DoosanAPI-blue?style=for-the-badge&logo=DoosanAPI&logoColor=white">







## 내가 기여한 부분


### 1. 시나리오 설계 및 opencv image 좌표 추출 알고리즘 제작

-image 레이아웃 후 skeleton 사용


### 2. Doosan API 중 구현 안 된 Nudge함수 제작



-본래 Doosan M0609 모델 사용시 제공되는 API 를 사용하지만 만들지 않은 API가 있어 함수로 구현


### 3. 코드 통합 및 디버깅



## 🌟핵심 기능 (코드로 보여주거나 코드 링크)







   
## 🎯트러블슈팅 경험  


1. 힘제어를 한 상태로 그림을 그려 진행하는 시나리오를 계획했지만, API가 안정화되지 않아 사용 불가능


2. opencv의 hough transform을 통해 라인 검출->좌표 추출을 하려했지만 좌표 추출 후 한붓그리기가 불가능한 형태의 좌표 순서가 나옴








## 🔨해결방법


1-1. 구현 가능한 API를 제작함. (Nudge), 
하지만 힘제어 함수는 구현하지 못해 미완성으로 남고 사용을 못 했다.



2-1. skeleton을 사용해 자동으로 라인의 좌표와 한붓그리기 시 좌표 순서가 지정되는 path를 만들음.




## 회고 / 느낀 점

-처음으로 6축 로봇팔 모델을 사용해서 manipulator와 비슷할거라 생각했는데 dart platform기반 로봇 동작이 되는 점이 신기했고 task build라는 기능을 통해 코드를 함수단위로 작성 하는 것의 중요성을 느낌

-이미지를 여러가지 방식으로 처리하며 opencv에 대한 이해도를 높일 수 있었고 추출된 좌표들을 부드럽게 연결하는 과정에서 path planning에 대한 고민도 할 수 있었음
