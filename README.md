# 🛩️ Airecon - AI-powered aerial monitoring system for industrial anomaly detection
# 제조업 현장의 이상 상황을 감지하는 자율주행 항공 감시 드론 시스템

![Airecon Demo](1st.gif)

## 📘 Overview

**Airecon**은 AI 기반 비전 인식과 자율비행 기술을 결합하여  
산업 현장의 이상 상황(화재, 침입, 설비 이상 등)을 실시간으로 탐지하고 대응하는  
**지능형 항공 감시 플랫폼**입니다.
### 산재예방이 점점 더 중요해지면서 이러한 산재들을 효과적으로 예방하기위해 프로젝트를 진행하였습니다.


## 🚀 Key Features

- 🤖 **AI Detection** — 실시간 영상 기반 이상상황 감지 (작업자 복장 확인, 쓰러짐 발견)
- 🛰️ **Autonomous Flight** — 경로 계획 및 비행 제어 (PX4 + ROS 2)
- 🧩 **V-SLAM** — 비전을 이용하여 매핑 후 경로생성
- ☁️ **On Dashboard** — 실시간 모니터링 및 데이터 기록


## 🧱 System Architecture

```text
Airecon
├── main (PX4, ROS 2, Rtabmap slam, YOLO)
│   ├── gimbal_control --> CCTV
│   ├── waypoint_flier --> Waypoint
│   └── px4msgtest --> V-SLAM 매핑
├── arucoland (OpenCV4.10)
    ├── aruco_detector
    ├── precise_land
```


## ⚙️ Tech Stack

| Category | Stack |
|-----------|-------|
| **Flight Control** | PX4, MAVSDK, ROS 2 Humble |
| **AI / Vision** | PyTorch, YOLOv12, OpenCV |
| **Communication** | uXRCE-DDS, MAVLink |
| **Backend** | Flask, Supabase, MySQL |
| **Frontend** | React, React Native |
| **Hardware** | Jetson Orin, Pixhawk, Depth Camera |

---


## 🧭 Roadmap

- [ ] Real-time anomaly detection (fire/smoke/person)  
- [ ] PX4 + ROS 2 autonomous patrol mission  
- [ ] Multi-drone coordination system  
- [ ] Cloud alert dashboard integration  
- [ ] Edge computing optimization (Jetson Orin)  

---

## 👥 Contributors

| Name | Role | Description |
|------|------|-------------|
| **Yoo (레게머핀)** | Founder / Lead Dev | System architecture, AI vision, PX4 integration |

---

## 📜 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---
