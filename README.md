# 🤖 Face Arm (Service Collaborative Robot)

> 
> **사용자 인식 및 객체 탐지 기반의 시니어 헬스케어 서비스 협동 로봇** > **두산로보틱스 지능형 로보틱스 엔지니어 과정 (K-Digital Training)** 
> 
> 

## 1. 프로젝트 개요 (Project Overview)

### 📅 프로젝트 기간

2025.05.23 ~ 2025.06.05 (총 10일) 

### 🎯 기획 의도

급격한 고령화 사회로의 진입에 따라 노인 복지 부담이 증가하고 있습니다. 이에 따라 시니어 타운(예: 삼성 노블카운티)과 같은 환경에서 **거동이 불편한 노약자나 장애인의 자립 생활을 돕고 삶의 질을 향상**시키기 위해, **얼굴 인식과 음성 명령을 통해 물건을 전달하는 보조 로봇**을 기획했습니다.

### 💡 기대 효과

**사회적 기여:** 고령자 및 장애인을 위한 생활 보조 및 자립 지원 

**산업적 활용:** 요양 시설의 보조 인력 부족 해소 및 비대면 서비스 환경 구축 

---

## 2. 팀 구성 (Team Members) 

| 이름 | 역할 | 담당 업무 |
| --- | --- | --- |
| **최정호 (팀장)** | 객체 인식 / PM | 객체 인식 모델(YOLO) 학습 및 발표, 전체 일정 관리 |
| **이하빈** | 얼굴 인식 / Vision | 얼굴 인식 모델 학습 및 2D-3D 좌표 리매핑(Remapping) 구현 |
| **이세현** | 로봇 제어 / LLM | 프롬프트 엔지니어링(LLM) 및 로봇 동작 제어(Control) 코딩 |
| **홍진규** | 예외 처리 / QA | 시스템 예외 처리 코드 작성 및 테스트 |

---

## 3. 개발 환경 (Development Environment) 

### 🛠 Hardware

* **Robot Arm:** Doosan Robotics M0609 
* **Gripper:** OnRobot RG2 Gripper 
* **Camera (Face):** Logitech C270 (RGB) 
* **Camera (Object/Depth):** Intel RealSense D435i (RGB-D) 



### 💻 Software & Tech Stack

| Category | Stack |
| --- | --- |
| **OS / Middleware** | Ubuntu, **ROS 2** |
| **Language** | **Python** |
| **AI / Vision** | **Ultralytics YOLO 11**, OpenCV |
| **Voice / LLM** | **OpenAI Whisper** (STT), **OpenAI API** (LLM), **Naver Clova** (TTS) |
| **Dataset** | Roboflow (데이터셋 관리 및 증강) |
| **IDE / Tools** | VS Code, Google Colab, Notion |

---

## 4. 시스템 아키텍처 (System Architecture) 

<img src="https://raw.githubusercontent.com/yabeen0126/PickandPlace/main/img/arc.png" width="720"/>

**Face_Yolo (C270):** 사용자 얼굴 인식 후 2D 이미지 좌표를 로봇 기준 3D 좌표로 변환하여 발행

**Object_Yolo (RealSense):** 객체 인식 및 Depth 센서를 활용한 거리(z축) 측정 

**Get_Keyword:** 사용자의 음성을 텍스트로 변환(STT)하고 LLM을 통해 핵심 의도(물건 이름, 행동) 파악 

**Robot_Control:** 수신된 좌표와 명령어를 종합하여 로봇 팔의 이동 및 그리퍼 제어 수행 

---

## 5. 주요 기능 및 핵심 기술 (Key Features)

### 1️⃣ 얼굴 인식 및 좌표 리매핑 (Face Detection & Remapping)

* **기능:** 사용자의 얼굴을 추적하여 로봇이 사용자의 위치로 물건을 가져다 줄 수 있도록 함.
* **기술:** Logitech C270 카메라의 2D 픽셀 좌표(x, y)를 로봇의 World 좌표계(x, z)로 변환.
* **로직:** 카메라 해상도와 실제 작업 공간의 크기를 고려한 선형 보간 리매핑 함수 구현.


### 2️⃣ 객체 인식 및 심도 측정 (Object Detection with Depth)

* **기능:** 지갑, 충전기, 우산 등 3가지 클래스의 물체를 인식하고 정확한 위치 파악.
* **기술:** YOLO 모델 탐지 결과(Bounding Box)의 중심 좌표를 추출하고, RealSense Depth Map을 매핑하여 실제 거리(z) 계산.
* **데이터셋:** Roboflow를 활용하여 클래스별 데이터셋 구축 및 Data Augmentation(Flip, Blur, Noise 등) 적용.



### 3️⃣ 음성 인식 및 의도 파악 (Voice Interaction)

* **기능:** "충전기 갖다 줘", "제자리에 놔" 등의 자연어 명령 수행.
* **기술:** OpenAI Whisper로 음성을 텍스트로 변환(STT)하고, OpenAI API(LLM)를 통해 프롬프트 엔지니어링으로 핵심 키워드(도구, 행동) 추출.
* **피드백:** Naver Clova Dubbing을 활용하여 "이동을 시작합니다", "물건을 잡아주세요" 등의 음성 안내 제공.



### 4️⃣ 안전 기능 및 예외 처리 (Safety & Exception Handling)

* **외력 감지:** 물건 전달 시 5초간 그리퍼의 외력을 감지하여 사람이 물건을 받으면 자동으로 놓음(Release).

* **예외 처리 테이블:** 
* `객체 인식 실패`: 5초간 미인식 시 작동 중지 및 로그 출력.
* `충돌 감지`: Force Sensor 임계치 초과 시 즉시 정지.
* `다중 사용자 감지`: 정확도(Confidence Score)가 높은 사용자를 우선순위로 설정.

---

## 6. 시연 시나리오 (Demo Scenarios) 

**1. "OOO 갖다 줘" (Fetch):** 물체 좌표 인식 -> 파지(Grip) -> 사용자 얼굴 좌표로 이동 -> 외력 감지 후 전달 -> 복귀.
<img src="https://raw.githubusercontent.com/yabeen0126/PickandPlace/main/img/01.gif" width="720"/>


**2. "여기로 와" (Come Here):** 사용자 얼굴 좌표 인식 -> 해당 위치로 이동 -> 대기.
<img src="https://raw.githubusercontent.com/yabeen0126/PickandPlace/main/img/02.gif" width="720"/>


**3. "위/아래/오른쪽/왼쪽/앞/뒤" :** 목표 방향 인식 -> 해당 위치로 이동 -> 대기.
<img src="https://raw.githubusercontent.com/yabeen0126/PickandPlace/main/img/03.gif" width="720"/>


**4. "내려놔 / 갖다 놔" (Return):** 물체 파지 -> 기억된 원래 좌표로 이동 -> 놓기(Release) -> 홈 위치 복귀.
<img src="https://raw.githubusercontent.com/yabeen0126/PickandPlace/main/img/04.gif" width="720"/>


**5. "돌아가" (Go Home):** 시야가 가려지거나 작업 종료 시 기본 설정된 홈 위치로 복귀.
<img src="https://raw.githubusercontent.com/yabeen0126/PickandPlace/main/img/05.gif" width="720"/>


