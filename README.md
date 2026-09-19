# 팀원 코드 검토 패키지 — Stage1 QR 제거 버전

**검토할 최신 코드는 `stage1_collection_noqr_20260919/`에 있다.** 이 폴더 전체를 전달하면 된다. 서버에 접속하지 않고 로컬에 보관된 파일만 모았다. 시뮬레이션 수집 코드는 패키징 과정에서 변경하지 않았다.

## 폴더 구성

| 경로 | 내용 |
|---|---|
| `stage1_collection_noqr_20260919/` | 최신 QR 제거 수집·분석 코드, 경로, 테스트, 상세 설명 |
| `stage1_collection_20260918/` | 비교 검사에 필요한 기존 QR 버전 원본 17파일. 최신 구현으로 사용하지 않음 |
| `support/stage1_scripts/` | 경로 보간, 카메라 배치·시각화, Stage1 제어·자원 관리 보조 코드 |
| `support/mro_scripts/` | LiDAR 원시 패킷 해석, PointCloud2 데이터 구성, 저장 용량 검사 |
| `support/config/` | 현재 경로, 카메라 배치·모델·TrackingCenter 설정, DDS 설정 |
| `support/qr_validation_reference/` | 과거 QR판 생성·판독 코드. 비교용이며 QR 제거 수집에서는 사용하지 않음 |
| `evidence/` | 기존 QR 5초 파일럿 결과와 배포·로컬 검사 근거 |
| `PACKAGE_CHECK.json` | 패키지 내 파일·설정·소스 구문 및 오프라인 검사 확인 결과 |
| `FILE_MANIFEST.json` | 파일별 원본 위치, SHA-256 및 패키징 중 변경 여부 |

## 권장 읽는 순서

1. 최신 코드 폴더의 `README.md`: 수집 범위와 시각 검증의 한계.
2. `record_runtime.py` → `record_render.py`: 시계 전진, 센서 렌더 완료 대기, 실제 센서 읽기.
3. `record_worker.py` → `record_common.py`: ROS 발행, 별도 DDS 수신·bag 저장, 토픽과 데이터 형식.
4. `record_lidar.py`와 `support/mro_scripts/`의 LiDAR 모듈: 원시 시각과 점 데이터 처리.
5. `stage1_scene.py`, `waypoints.json`, `support/stage1_scripts/waypoint_motion.py`, `support/config/`: 카메라·드론·경로 구성.
6. `analyze_bag.py` → `timing_assessment.py`: 저장 자료의 메타데이터 검사와 합격 판정 범위.
7. `run_record.py`, `record_driver.py`: 서버 환경에서 수집을 시작·감시·종료하는 흐름.
8. `CHANGES.patch`, 로컬 검사 결과, `evidence/`: 변경 내용과 기존 실험 근거.

## 인터넷·서버 없이 실행할 수 있는 검사

Python 3.10 이상에서 이 패키지의 최상위 폴더를 기준으로 실행한다. 아래 검사는 Python 표준 라이브러리만 사용하며 SSH·Isaac Sim·ROS를 시작하지 않는다.

```text
python -B stage1_collection_noqr_20260919/verify_local.py
```

이전 버전 소스도 패키지에 포함했으므로 다른 작업 폴더 없이 비교할 수 있다. 이 명령은 최신 코드 폴더의 `LOCAL_VERIFICATION.json`을 갱신한다. `FILE_MANIFEST.json`은 전달 시점의 체크섬이므로 이 보고서를 다시 생성하면 해당 보고서의 해시는 달라진다. `ORIGIN.json`의 상대 경로 구분자만 패키지에서 `/`로 정규화했다. 수집 동작은 수정하지 않았다.

## 실행 환경과 현재 상태

- 이 패키지는 **소스 검토와 오프라인 검사용**이다. Isaac Sim 설치본, USD 장면·드론·격납고·Ouster 에셋, 원본 rosbag·영상, Python 바이너리·제3자 라이브러리는 포함하지 않는다.
- 시뮬레이션 재현에는 기존 Linux Stage1 환경과 Isaac Sim 5.0의 `omni`·`pxr`·센서 확장, ROS 2 Humble bridge/rclpy, NumPy, rosbags 0.11.5, 기존 `mro_runtime` 기반 시설 등이 필요하다. 과거 QR 분석에는 OpenCV도 사용했다. 이것은 별도 환경에서 설치·실행까지 확인한 배포 패키지가 아니다.
- `support/`는 검토를 돕도록 모아 둔 원본 사본이다. 코드의 서버 절대 경로와 import 배치는 원래대로 유지했으며 자동으로 `support/`를 실행 경로에 추가하지 않는다.
- `ops.py`, `deploy.py`, `run_record.py`, `run_analysis.py`, `run_replay.py`는 서버 운용용 소스다. 이 패키지에서 실행하지 않았다. 기존 WSL/SSH 도우미·서버 설치 구조를 전제로 하며, 인증 자료·SSH 설정은 포함하지 않았다.
- QR 제거 버전은 **서버 미배포·실제 수집 미실행**이다. 카메라/드론 QR와 색 평판은 수집 코드에서 사용하지 않는다. 시계·헤더 생성 방식과 렌더 완료 순서는 이전 버전과 유지했다.
- QR 제거 버전의 실제 영상 내용 시각은 아직 독립 검증하지 않았다. LiDAR 점별 취득 시각도 기존 미검증 상태다. 30초 본 수집은 아직 실행되지 않았고, 기존 route 실행 제한도 유지한다.
- `evidence/`의 5초 파일럿은 **QR가 있던 이전 버전**의 결과다. 일부 문서의 실행·SSH 연결 상태는 당시 기록이며 현재 상태가 아니다. 마지막 확인 상태는 2026-09-18 Isaac Sim 종료 및 SSH 연결 종료다.

파일 출처와 검증 범위는 `FILE_MANIFEST.json` 및 `PACKAGE_CHECK.json`에 기록했다. 기존 작업 폴더와 서버 데이터는 변경하지 않았다.
