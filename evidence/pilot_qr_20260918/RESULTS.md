# Stage1 원본 RGB·LiDAR rosbag2 파일럿 결과 — 2026-09-18

**원본 해상도 카메라 3대 + Ouster LiDAR + 드론 GT의 5시뮬초 통합 수집과 저장·제한 재생 검증을 완료했다. 전체 센서의 취득 시각 동기화는 아직 미확정이며 30초 본 수집은 실행하지 않았다.**

| 항목 | 실제 결과 |
|---|---|
| 기록 시간 | 실제 글로벌 시각 0.033333335072 → 5.033333595842s, 경과 5.000000260770s |
| 원본 RGB | cam_01·03 각각 3840×2160, cam_02 5320×4600, RGB8 각 76장 = 총 228장 |
| 드론 GT·clock | 시작·끝 포함 각각 301표본, 60시뮬Hz |
| LiDAR | native 패킷 301개, 유효 점 793,014개. 32ch·10Hz·512 프로필; 패킷을 완성 회전 스캔으로 부르지 않음 |
| bag | 14토픽, 2,493메시지, 9,398,600,330bytes ≈ 8.75GiB |
| 실제 처리시간 | 준비·수집·drain·정리 포함 324.9초. 실시간 15FPS 성능을 의미하지 않음 |
| 전달·저장 | 발행 수 = 별도 DDS 수신 수 = bag 메시지 수, SQLite quick_check 정상 |
| 제한 재생 | 14토픽의 처음·마지막 표본 총 25개를 native rclpy로 재발행, 별도 구독자가 저장 CDR와 바이트 단위 일치 |
| 늦은 구독 | TF·session·waypoints 발행 후 시작한 구독자도 3개 정적 메시지를 정확히 수신 |

bag 서버 경로:
`/mnt/DATA/workspace/ws_minho/mro_1/stage1/outputs/collection_record_pilot01_be59cd4fd7/bag`

SQLite SHA-256: `80970cc130d1e6e8aac026bd75d5d3b9d307b902a05ce0df666872cde6d38904`. 원본은 읽기 전용·immutable 모드로 검사했다. 로컬에는 분석 JSON과 마지막 원본 이미지에서 추출한 PNG 3장만 가져왔다. PNG는 수집 후 확인용이며 녹화의 중간 입력이 아니다.

이미지 내용의 시각은 예상 코드나 GT 위치를 주지 않은 독립 QR 판독으로 판정했다. 드론 장착판 **76/76장(cam_01)**이 실제 글로벌 시각·Image.header와 일치했다. 카메라 기준판은 145개 유효 판독이 모두 일치했고 83개는 판독 불가였다. 합계 221개 유효 판독에서 시각 불일치는 0개다. 카메라 native simulationTime·ReferenceTime 조회와 이미지 헤더의 수치 차이는 228장 모두 0ns였다. 이 값은 저장된 수치의 일치이며 물리 노출의 나노초 정밀도 보증이 아니다.

| 카메라 | 이미지 | 기준판 판독 | 기준판 불가 | 드론판 판독 | 드론판 불가 | 유효 시각 불일치 |
|---|---:|---:|---:|---:|---:|---:|
| cam_01 | 76 | 72 | 4 | 76 | 0 | 0 |
| cam_02 | 76 | 32 | 44 | 0 | 76 | 0 |
| cam_03 | 76 | 41 | 35 | 0 | 76 | 0 |

cam_02·03에서는 이 경로의 첫 5초 동안 드론판을 판독하지 못했다. 기준판도 일부 후반 구간에 연속 판독 불가가 있어 모든 카메라의 모든 이미지 내용을 직접 인증했다는 뜻은 아니다. 사용자 기준대로 판독 불가를 시각 불일치로 세지 않으며, 판독률 100%를 위한 재수집을 하지 않았다. 이번 기준판은 cell 12px이고 이전 저해상도 최종 시험은 16px이었다. 이 설정 차이를 보존하며, 후속 수집에서 판독성 개선을 한다면 먼저 기존 16px 설정을 재사용할 수 있다. 판독 임계값이나 코드를 보정하지 않았다. 이미지 시각 판정에는 위치 오차나 재투영 오차를 사용하지 않았다.

LiDAR는 원시 timestampNs, signed timeOffsetNs, 점별 uint64 시각(lo/hi UINT32), native 원소 인덱스, 프레임 시작·끝 시각/pose, 현재 관측 글로벌 시각을 분리하여 보존했다. 모든 793,014점의 이진 시각 복원과 payload hash를 다시 확인했다. 이 연속 구간의 native packet header−global 관측 시각은 1,416,666,740..1,416,666,741ns였다. 다른 시작 세션이나 pause 구간에도 적용되는 보정 상수가 아니다.

유효 점 2,099개(0.265%)가 자신의 native 프레임 시각 범위를 벗어났다. 이 현상이 있는 패킷은 100개다. 모든 native 원소 기준 최대 끝 범위 초과는 127,139ns = 0.127139ms다. 이것만으로 센서 오류라고 단정하지 않는다. firing 묶음의 경계 의미와 글로벌 취득 구간을 아직 검증하지 않았으므로 `lidar_acquisition_time_verified=false`를 유지했다. 범위 밖 점을 버리거나, 원시 헤더를 글로벌 시각으로 덮어쓰거나, deskew 완료로 표시하지 않았다. 첫 정지 상태 패킷도 실제 관측값 그대로 남겼다.

별도 pause/re-enable 시험에서는 글로벌 시계가 정지해도 LiDAR native 시계가 진행했으며, RenderProduct off/on 후 native 프레임 번호가 초기화됐다. 따라서 nativeframe×고정 dt나 전체 세션의 단일 offset을 일반 해법으로 적용할 수 없다. 짧은 연속 구간의 프레임 pose 일치는 LiDAR 메타데이터 일관성 확인이며 점별 취득 시각 인증이나 카메라 위치 기반 시각 검증으로 쓰지 않았다.

수집기는 80MiB 단일 메시지 한도, 256MiB 바이트 큐, RGB QoS depth 2/작은 토픽 depth 128을 사용한다. 실제 큐 최대는 98,300,622bytes였다. 매 프레임 디스크 ACK나 PNG 중간 파일 없이 native RGB → IPC → rclpy → 별도 DDS 구독 → rosbag2 SQLite로 저장한다. 정적 토픽의 transient_local QoS도 bag 메타데이터에 기록했다. bag record time은 DDS 수신 wall time이고, 이미지/GT 취득 시각은 헤더·clock·mapping으로 읽는다.

초기 연속 렌더 경로는 원본 영상 QR가 헤더보다 4스텝, 동기 렌더 설정 후에도 1스텝 늦어 실패 자료로 보존했다(native01~03). 기존 frozen Replicator만 사용한 native04는 카메라 시각은 맞지만 LiDAR가 빈 패킷이었다. 최종 경로는 일반 render의 유효 LiDAR 출력을 NEW_FRAME에서 보존하고, 물리 시계를 전진시키지 않는 기존 Replicator 완료 대기로 카메라를 읽는다. native05에서 원본 12장의 source RGB hash까지 실제 bag과 일치했고, 5초 파일럿에서는 228장·301패킷의 수와 내용·시각을 사후 검증했다. 모든 이전 실패와 bag을 삭제하지 않았다.

합의한 좌표 `(5,0,.1665)→(5,0,5)→(5,10,5)→(5,0,5)→(5,0,.1665)`와 Ouster 장착을 Stage1 영구 구성에도 반영했다. 29.667초 경로의 9개 waypoint/중간점과 저장 후 재개방을 확인했다. 5초 파일럿은 상승 후 첫 수평 이동 일부까지 포함한다. 전체 왕복 경로는 이번에 녹화하지 않았다. 실제 추력 비행·로버·IMU·Odometry는 범위에 없다. 과거 scene/override/config/source/release는 `scene_revision.json`의 archive에 보존했다.

이제 남은 필수 확인은 **LiDAR native 점 시각의 경계 의미와 글로벌 취득 구간 매핑**이다. 이 확인을 통과하기 전에는 30초 수집을 동기 데이터셋 합격으로 진행하지 않는다. 공식 `ros2 bag info/play` 명령의 대상 환경 검증도 남아 있다. 설치된 Humble bridge에는 rclpy·ros2cli 기반 기능이 있으나 bag 명령 패키지는 확인되지 않아, 이번에는 실제 DDS 제한 재생으로 호환성을 확인했다. 이 둘을 같은 검증으로 주장하지 않는다.

파일럿 자원 표본에서 GPU1 최소 여유 36,075MiB, 최대 65°C, CPU 최대 사용 15.0%였다. 최종 공유 디스크 여유 2.906TiB이며 원래 누적 2TiB/최소 여유 1.13TiB 기준선을 유지했다. 기존 파일 삭제와 다른 작업 중단은 없었다.

최종 2026-09-18T11:08:00.028077+00:00 상태는 새 Stage1 launch `20260918T110719Z_a82b5d9bbb`, Kit2853157/start279857781, supervisor2853138이다. GPU1, timeline0 일시정지, recorder·분석·ROS worker 없음. 영구 Stage1 LiDAR는 켜진 구성이며 수집용 RenderProduct는 실행 중이 아니다. 원본23파일과 release34파일 hash, 실제 RO 루트·RO mro_1·RW Stage1 mount와 로그/캐시 경로를 확인했다. SSH Master325는 연결 상태로 남겼으며 추가 수집/자동 재시도는 예약하지 않았다.

| 저장 토픽 | 메시지 수 |
|---|---:|
| `/stage1/session` | 1 |
| `/stage1/drone/waypoints` | 1 |
| `/tf_static` | 1 |
| `/clock` | 301 |
| `/stage1/drone/gt_pose` | 301 |
| `/stage1/drone/gt_velocity` | 301 |
| `/stage1/drone/lidar/points_raw` | 301 |
| `/stage1/time_mapping` | 830 |
| `/stage1/cameras/cam_01/image_raw` | 76 |
| `/stage1/cameras/cam_01/camera_info` | 76 |
| `/stage1/cameras/cam_02/image_raw` | 76 |
| `/stage1/cameras/cam_02/camera_info` | 76 |
| `/stage1/cameras/cam_03/image_raw` | 76 |
| `/stage1/cameras/cam_03/camera_info` | 76 |

주요 근거: `pilot01_analysis.json`, `pilot01_reports.json`, `pilot01_replay.json`, `pilot01_lidar_time_audit.json`, `persistent_scene_readback.json`, `FINAL_STATE.json`, `SUMMARY.json`. 렌더 순서 참고: [NVIDIA Isaac Sim 5.0 예제](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/replicator_tutorials/tutorial_replicator_isaac_snippets.html), [GMO 구조](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/docs/source/generic_model_output/generic_model_output.html). 실제 설치된 5.0 코드와 실측을 우선했다.
