# 🛩️ Airecon - AI-powered aerial monitoring system for industrial anomaly detection
# 제조업 현장의 이상 상황을 감지하는 자율주행 항공 감시 드론 시스템



## 📘 Overview

**Airecon**은 AI 기반 비전 인식과 자율비행 기술을 결합하여  
산업 현장의 이상 상황(화재, 침입, 설비 이상 등)을 실시간으로 탐지하고 대응하는  
**지능형 항공 감시 플랫폼**입니다.


## 🚀 Key Features

- 🤖 **AI Detection** — 실시간 영상 기반 이상상황 감지 (화재, 연기, 인체, 장비 이상 등)
- 🛰️ **Autonomous Flight** — 경로 계획 및 비행 제어 (PX4 + ROS 2)
- 🧩 **Modular Architecture** — 비전, 제어, 통신 모듈로 구성된 확장형 시스템
- 🏭 **Industrial Integration** — 공장 내 IoT 센서 및 제어시스템 연동
- ☁️ **Cloud Dashboard** — 웹 기반 실시간 모니터링 및 데이터 기록


## 🧱 System Architecture

```text
Airecon System
├── Drone Layer (PX4, MAVSDK, ROS 2)
│   ├── Flight Control
│   ├── Sensor Fusion
│   └── Mission Planner
├── AI Vision Layer (Jetson / OpenCV / YOLO)
│   ├── Anomaly Detection
│   ├── Object Tracking
│   └── Event Trigger
└── Cloud Layer (Flask / Supabase / Dashboard)
    ├── Data Logging
    ├── Alert System
    └── Visualization
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
