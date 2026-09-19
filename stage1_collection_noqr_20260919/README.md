# Stage1 QR 제거 수집 버전 — 2026-09-19

로컬에서 작성한 QR 제거 버전이다. 서버에 접속하거나 배포하지 않았고 Isaac Sim으로 실행하지 않았다. 기존 `stage1_collection_20260918` 코드와 수집 데이터는 그대로 보존했다. 마지막 서버 상태는 2026-09-18 사용자 요청에 따른 Isaac Sim 종료·SSH 연결 종료다.

드론 부착 QR판과 카메라 화면의 기준 QR판을 모두 생성하지 않는다. QR 셀 업데이트와 온라인·사후 판독도 제거했다. 색 평판은 계속 꺼져 있다. 기존 장면에 활성 QR판이 남아 있으면 수집을 거부한다. 새로 열린 Stage1 장면을 사용해야 한다.

## 타임스탬프에 대해 유지한 것

- 글로벌 시각은 기존과 동일하게 SimulationManager에서 실제 관측한다. 60Hz로 한 스텝씩 전진하며 카메라는 4스텝마다 읽는다.
- 카메라 Image·CameraInfo, 드론 GT 및 `/clock`의 헤더 생성·발행 방법을 유지했다. 카메라 native simulationTime, ReferenceTime 및 `/stage1/time_mapping`도 보존한다.
- LiDAR NEW_FRAME 취득 → 글로벌 시계를 전진시키지 않는 Replicator 카메라 완료 대기 → RGB 읽기 순서를 유지했다. 초기화와 30회 예열도 유지한다.
- LiDAR native 헤더·점별 원시 시각, DDS 수신 wall time을 그대로 기록한다. LiDAR 시각 보정은 추가하지 않았다.
- 원본 해상도 3카메라, GT 60Hz, 이미지 15시뮬Hz, 14토픽, 기존 왕복 경로를 유지한다. 30초는 1,800스텝이며 시작·끝 포함 카메라마다 451장이다.

따라서 **타임스탬프 규칙과 수집 순서는 QR 버전과 같다**고 소스 수준에서 확인할 수 있다. 별도 세션의 절대 시작 시각, wall time, 실행 속도까지 동일하다는 뜻은 아니다.

**QR 없는 실제 영상의 내용 시각이 동일하게 맞는지는 아직 실행 검증하지 않았다.** QR은 시계가 아니라 영상에 보이는 시각 표식이었다. 제거하면 렌더링 부하·장면 내용이 바뀌며, 메타데이터가 일치해도 오래된 RGB가 섞였는지 독립 판독할 수 없다. QR 제거에 따라 가림·반사와 LiDAR의 검출 점도 달라질 수 있다. 기존 QR 시험은 기존 버전의 근거로 보존하며 새 버전의 실측 합격으로 재사용하지 않는다.

## 결과 판정

`analyze_bag.py`는 원본 bag을 읽기 전용으로 열고 토픽 수·전달 수·시계 단조성·이미지 시퀀스·헤더/시계 매핑·native/ReferenceTime 수치·LiDAR 원시 payload와 점 시각을 검사한다. QR가 없는 이미지를 판독 불가로 세거나 QR 검증을 통과했다고 기록하지 않는다.

| 필드 | 의미 |
|---|---|
| `camera_metadata_time_pass` | 저장된 시각 메타데이터 및 샘플 순서 검사 결과 |
| `camera_content_time_status` | `not_checked_qr_removed` |
| `camera_content_time_verified` | `null`: 이번 영상 내용의 시각은 독립 검증하지 않음 |
| `camera_time_pass` | `null`: 기존 QR 포함 종합 판정은 적용하지 않음 |
| `lidar_acquisition_time_verified` | `false`: 기존 점별 취득 시각 미검증 상태 유지 |
| `synchronized_dataset_pass` | `false`: 전체 센서 취득 시각 합격으로 표시하지 않음 |

## 코드 검토 순서

1. `record_runtime.py`: 장면 준비, 시계 전진, 렌더 완료, 센서 읽기 및 IPC.
2. `record_render.py`: 렌더 완료 이벤트와 annotator 평가. 기존 버전과 바이트 단위 동일하다.
3. `record_worker.py`: native rclpy 발행, 별도 DDS 구독, rosbag2 SQLite 저장. 기존 버전과 바이트 단위 동일하다.
4. `analyze_bag.py`와 `timing_assessment.py`: QR 없는 자료의 검증 범위와 판정.
5. `waypoints.json`, `stage1_scene.py`: 이동 경로와 장면. 기존 버전과 바이트 단위 동일하다.
6. `CHANGES.patch`, `ORIGIN.json`, `LOCAL_VERIFICATION.json`: 기존 버전과의 차이 및 로컬 검사 근거.

Isaac Sim·USD·ROS 2 라이브러리, 장면/센서 에셋, 기존 Stage1 supervisor·좌표 보조 모듈은 이 폴더에 포함된 독립 설치본이 아니다. `record_common.py`의 원격 경로는 기존 Stage1 설치 위치를 가리킨다. 소스 검토는 로컬에서 가능하며 실제 실행에는 해당 환경이 필요하다.

## 로컬 검사와 향후 실행

`python -B verify_local.py`는 네트워크나 Isaac Sim 없이 구문·원본 해시·수집 루프·렌더/발행 소스 동일성과 합성 시각 회귀를 검사한다. 실제 30초 영상을 생성하거나 시간 동기화를 실측하는 시험은 아니다.

`ops.py`, `deploy.py`, `run_record.py`, `run_analysis.py`, `run_replay.py`는 기존 서버 운용용 코드의 사본이다. 로컬 검토 과정에서는 실행하지 않았다. 향후 사용자가 서버 재접속을 요청하면 자원 확인, 기존 owned 세션 종료 확인, 배포 전 archive와 해시 검증, 새 세션 시작 절차를 거쳐야 한다. 새 launcher는 수집 전 서버 소스가 이 QR 제거 버전과 일치하는지 확인한다.

30초 route 모드의 기존 `record_driver.py` 실행 제한도 변경하지 않았다. `synchronized_dataset_pass=true`를 요구하므로 현재 저장된 acceptance로는 route 수집이 차단된다. 원시 자료 수집 허용과 전체 동기화 검증 합격을 구분하는 실행 정책 정리는 별도 후속 작업이다. 이번 QR 제거 변경을 이유로 검증 상태를 true로 바꾸지 않는다.

서버 재사용 시에는 먼저 QR 없는 짧은 시험으로 실제 QR 부재, 정상 렌더 완료, 누락·중복 없는 시퀀스, 시각 메타데이터 일치와 자원을 확인하는 것이 적절하다. 이는 새 버전의 기본 실행 검사이며, 이미지 내용 시각에 대한 QR 기반 직접 증명과는 검증 범위가 다르다.
