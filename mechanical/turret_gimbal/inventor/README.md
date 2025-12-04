# Turret Gimbal Mechanical Design (Autodesk Inventor)

본 디렉토리는 **3축 짐벌 기반 자동 조준 터렛**의 기계설계 자료를 정리한 폴더입니다.
전체 프레임, 3축 링크(Pitch/Roll/Yaw), 기어(Design Accelerator), LIDAR·배터리·STM32·L298N 마운트 등 **전체 구조를 직접 설계**한 프로젝트이며, Pack&Go 기반으로 모든 Inventor 파일을 계층별로 재구성하였습니다.

---

## 1. Folder Structure

```
inventor/
 ├─ turret_V1.ipj                     # Inventor 프로젝트 루트
 ├─ packngo.txt                  # Pack&Go 파일 구조 정보
 ├─ Design Data/                      # Inventor 자동 생성 데이터
 ├─ Templates/                        # Inventor 템플릿
 ├─ OldVersions/                      # 이전 버전 아카이빙
 ├─ 01_parts/                         # 단품(ipt)
 ├─ 02_subassemblies/                 # 축·기어·케이스 조립(iam)
 ├─ 03_main_assembly/                 # 최종 조립(turret_V1.iam)
 └─ 05_exports/                       # stl/step/urdf 등 외부 내보내기
```

---

## 2. Project Goal

* **3축 짐벌 기반 자동 조준 터렛**
* Roll–Pitch–Yaw 3축 구조로 총구 방향을 고속·정밀하게 제어
* LIDAR, 카메라, IMU 기반 자동 조준 및 추적 시스템과 결합 예정
* ROS2 기반 터렛 제어 및 서보/브러시 모터 신호 대응

---

## 3. Main Assembly

### ● `03_main_assembly/turret_V1.iam`

* 전체 Roll–Pitch–Yaw 축 통합
* 전동총 탑재 프레임
* 배터리, LIDAR, STM32, Arduino Mega, L298N 드라이버 고정부 포함
* Trigger용 MG90S 서보 포함

---

## 4. Sub Assemblies Overview

(세부 구조는 Pack&Go 기반 구조를 따르며, 축별 구성은 아래 기준으로 설계됨.)

### ● **Yaw Axis**

* 구동: DS35MG-180 (35kgf·cm)
* 기어 구조: Sun 50T / Planet 20T / Ring 90T
* 기어 모듈: m=2
* 하우징과 베이스 연결 구조
* LIDAR 및 상부 프레임을 지지

### ● **Pitch Axis**

* 구동: DS35MG-180
* 기어비: **1:2 (m=1.5)**
* 상부 총구 프레임을 지지
* 거친 동작 대비 베어링·축 강성 고려

### ● **Roll Axis**

* 구동: DS35MG-180
* 기어 구조: **베벨기어 0.8:1 (m=1.25)**
* 총구의 기울어짐 보정 역할
* 중량중심(CG)에 가깝게 배치하여 토크 최소화

---
## 우선행동

1. 아래 **05_exports 설명만** 필요한 형태로 정리해줄게.
2. 그대로 README에 붙여넣으면 됨.

---

# `05_exports/` 폴더 설명

```
05_exports/
 ├─ *.stl
 ├─ *.step
 └─ *.urdf
```

### ● 목적

Inventor 원본(ipt/iam) 외의 **외부 활용용 중립 파일**을 저장하는 폴더.
시뮬레이션, 3D프린팅, 타 CAD 호환 등을 위한 파일을 통합 관리.

### ● 구성

* **STL (.stl)**

  * 3D 프린팅 용도
  * Slicer(Cura 등)에서 바로 사용 가능
  * 세부 가공 공차(0.6mm) 반영 가능

* **STEP (.step / .stp)**

  * 타 CAD(Fusion, SolidWorks 등) 호환 목적
  * 구조 검토/공유에 최적화된 중립 포맷
  * 내부 기계 구조를 팀원·외부 협업자와 공유하기 좋음

* **URDF (.urdf)**

  * ROS2용 로봇 모델
  * RViz, Gazebo, Nav2, 시뮬레이션 환경에서 활용
  * 실제 Pitch/Roll/Yaw 축의 joint 정의 포함 가능

### ● 사용 기준

* Inventor 원본은 **inventor/** 내부에서 관리
* 프린팅/시뮬레이션/외부 호환이 필요한 파일만 **05_exports/**에 저장
* Git 용량 관리 차원에서 큰 파일은 Release에도 업로드 권장
---

## External Components (Mounted by CAD)

* **LIDAR X4 Pro**
* **Arduino Mega**
* **STM32**
* **L298N Motor Driver**
* **MG90S Trigger Motor**
* **배터리 팩**
* **DC Converter / Wire aisle 구조**

모든 장착 부품은 고정부(Bracket)와 마운트가 개별 단품(ipt) 또는 서브조립(iam)으로 구현됨.

---

## 6. Design Basis (하중·공차·재질 기준)

* 장착 하중: **약 2.5kg 전동총(전동 M4A1/RIS 스케일)**
* Pitch 요구 토크: **3.68 N·m**
* Yaw 요구 토크: **약 0.23 N·m**
* Roll 축은 중심부 근처로 배치하여 토크 감소
* 3D 프린팅 재질: **PLA**
* 설계 공차: **0.6 mm** 기준
* 간섭 발생 방지: 각 축 주변 베어링·하우징 충분 간극 확보
* 내부 배선(wire way) 통로 확보를 설계 기준에 포함

---

## 7. How to Open (재현 방법)

1. **Autodesk Inventor 실행**
2. `inventor/turret_V1.ipj` 프로젝트 열기
3. 메인 조립품:
   `03_main_assembly/turret_V1.iam` 실행
4. 서브조립품은 `02_subassemblies/`에서 개별 확인

---

## 8. Current Status / Todo

### ✔ 현재 완료 (V1)

* 전체 3축 짐벌 구조 Pack&Go 정리
* DS35MG 기반 3축 기어 구조 구현
* 전체 프레임, 배터리·LIDAR·STM32 마운트 완성
* Trigger MG90S 연동 구조 설계

### 🔧 예정 개선

* 케이블 길 여유 확보
* LIDAR Base 및 DC converter 지지 구조 보강
* 카메라·IMU 센서 하우징 설계
* Pitch 축과 Planetary Ring 결합 구조 개선
* Roll–Pitch–Yaw 기어비 최적화

---

## 9. Purpose of Documentation

이 README는 다음 정보를 팀원이 빠르게 파악할 수 있도록 한다.

* 설계 범위 및 목표
* 3축 기계 구조 개요
* 기어비와 모터 스펙
* Inventor 프로젝트 재현 방법
* 폴더 구조 및 부품 구성
* 향후 개선 방향

---

## 10. License / Usage

본 자료는 SAFAS 프로젝트 내부에서만 사용한다.

---

## 3줄 요약

* 위 README는 네가 제공한 모든 정보를 반영한 **최종 완성본**이다.
* 바로 repo에 넣으면 “팀원이 이해할 수 있는 CAD 구조 설명서” 역할을 한다.
* 파일 구조 캡처가 필요하면 README에 그림까지 넣는 버전도 만들어줄 수 있다.
