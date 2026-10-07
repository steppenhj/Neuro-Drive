# Neuro-Drive: 분산 UGV 제어 시스템 (RPi 5 + STM32)

![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.11-3776AB?logo=python&logoColor=white)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)
![STM32](https://img.shields.io/badge/MCU-STM32F411RE-03234B?logo=stmicroelectronics&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/MPU-Raspberry_Pi_5-C51A4A?logo=raspberrypi&logoColor=white)

라즈베리파이 5(Linux)와 STM32(FreeRTOS)가 역할을 나눠 움직이는 Ackermann 조향 RC카입니다.
RPi 는 웹 조종 · 모드 관리 · 경로 기록을 맡고, STM32 는 모터 · 엔코더 · 안전 정지를 맡습니다.
명령이 끊기면 두 쪽이 각자 차를 세우고, STM32 펌웨어는 직접 만든 UART 부트로더로 교체합니다.

---

## 시연 영상

**Phase 4 — Return-to-Home**

https://github.com/user-attachments/assets/2779ef3e-39d6-4a21-8bef-ed63d195250f

**Phase 5 — OTA 펌웨어 업데이트**

https://github.com/user-attachments/assets/81a38263-ff0c-47a8-944d-1e0a582e165a

---

## 구조

![System Architecture](assets/omd_diagram.png)

| 층 | 보드 | 하는 일 |
|---|---|---|
| 미션 (soft real-time) | Raspberry Pi 5 · Linux | 웹 UI · WebSocket 서버 · C++ 제어 코어(100Hz) · RTH 경로 기록·재생 · 명령 감시 |
| 반사 (hard real-time) | STM32 NUCLEO-F411RE · FreeRTOS | UART 수신(ISR + Queue) · 모터 PWM · 엔코더 · 안전 태스크 |

Linux 는 스케줄링이 결정적이지 않아 모터 제어를 맡기기 어렵습니다. 그래서 판단은 RPi, 구동은 STM32 로 나눴습니다.

```
Browser ──WebSocket(JSON)──▶ Python (Flask-SocketIO)
                                 │
                                 ├──UDP 12B (throttle, steering, mode)──▶ C++ Control Core
                                 │                                            │
                                 │                                       UART 115200
                                 │                                            ▼
                                 │                                    STM32 (FreeRTOS)
                                 │                                     ├─ UART RX ISR → Queue
                                 │                                     ├─ Motor Task (PWM)
                                 │                                     ├─ Encoder Task (TIM1)
                                 │                                     └─ Safety Task
                                 │
                                 └──UDP (엔코더 텔레메트리) ◀── C++ ◀──UART── STM32
```

---

## 진행

| Phase | 내용 |
|:-:|---|
| 1 | RPi 하나로 제어 — Python 웹 서버와 C++ 제어 프로세스 |
| 2 | STM32 를 붙여 판단(RPi)과 구동(STM32)을 분리 |
| ⚠️ | 실내 주행 중 충돌로 빠진 7.4V 배터리 선이 RPi 3.3V GPIO 에 닿아 GPIO 일부를 잃음 → 물리적 절연 · RTOS 페일세이프 · 결정적 C++ 제어 루프로 재설계 |
| 3 | FreeRTOS + ISR + Queue — 블로킹 UART 수신이 100Hz 제어 주기를 잡아먹던 구조를 바꿈 · P 제어 + 조향 비례 피드포워드 |
| 4 | Return-to-Home — 워치독과 자율 복귀의 충돌을 Keep-Alive 로 해결 · 명령 패킷 8B → 12B(mode 추가) |
| 5 | OTA — 직접 만든 UART 부트로더 · CRC 검증 · 섹터 관리 · 2026-09 유효 표식 추가 |
| 6 | CAN 2.0 — 별도 저장소 [multi-mcu-can](https://github.com/steppenhj/multi-mcu-can) 에서 2노드로 구현 |

---

## 안전 정지

- **두 층 워치독** — 브라우저는 100ms 마다 명령을 보냅니다. C++ 제어 코어는 500ms 동안 명령이 없으면 정지 명령을 보내고, STM32 의 안전 태스크도 같은 500ms 를 50ms 주기로 확인해 RPi 쪽이 통째로 죽어도 차를 세웁니다(최악 약 550ms).
- **정지(fail-stop)를 기본으로** — 통신이 끊기면 스스로 돌아오지 않고 멈춥니다. 지상 차량은 멈춘 상태가 안전 상태이고, 엔코더 추측 항법은 오차가 쌓여 감독 없는 복귀를 믿기 어렵습니다.
- **Return-to-Home 의 감시 범위** — 복귀는 조작자가 버튼으로 시작합니다(기록 → 복귀 → 해제). 복귀 중에는 RPi 안의 서버가 100ms 마다 keep-alive 를 보내므로, 서버 프로세스가 죽으면 멈추지만 **브라우저(Wi-Fi) 단절은 잡지 못합니다.** 복귀 중에도 브라우저 쪽을 감시하는 것이 남은 과제입니다.
- **경로를 RPi 에 둔 이유** — 경로는 엔코더 변화량과 조향각 한 쌍(8바이트)을 주행 중 100Hz 로 쌓아 계속 자랍니다. MCU SRAM(128KB)에 두면 수십 초~수 분이면 넘치므로 메모리가 큰 RPi 에 두고 역순으로 재생합니다.

---

## OTA 부트로더 (Phase 5)

| 영역 | 주소 | 크기 |
|---|---|---|
| 부트로더 | Sector 0 · `0x08000000` | 16KB |
| 앱 | Sector 1~7 · `0x08004000` | 최대 496KB − 256B |
| 유효 표식 | Sector 7 마지막 16바이트 · `0x0807FFF0` | magic · size · crc · ~magic |

1. 리셋 후 3초 동안 UART 로 `UPDATE` 를 기다리고, 안 오면 앱 검사로 넘어갑니다.
2. 업데이트 — 크기 수신 → Sector 1~7 지우기 → 256B 청크마다 ACK(stop-and-wait) → 다 쓴 뒤 **Flash 에서 다시 읽어** CRC32 비교.
3. CRC 가 맞을 때만 유효 표식을 씁니다. magic 을 마지막에 써서, 표식을 쓰다 끊겨도 무효로 남습니다.
4. 부팅 때 표식과 Flash CRC 를 다시 확인한 뒤에만 앱으로 넘어갑니다. 아니면 `NO_APP` 을 보내고 업데이트를 기다립니다.

부트로더를 앱과 나눈 이유는 실행 중인 섹터를 스스로 지울 수 없고, 앱이 어떻게 깨져도 다시 올릴 수 있어야 하기 때문입니다.

**보드 시험 (2026-09-28 · NUCLEO-F411RE)**

| 시험 | 결과 |
|---|---|
| 정상 전송 | `DONE` → 재부팅 → 앱 실행 |
| CRC 를 틀리게 전송 | `NACK` → 앱으로 넘어가지 않고 부트로더에서 대기 |
| 전송 도중 중단 | 반쯤 쓰인 이미지로 넘어가지 않음 → 다시 보내 복구 |

**한계** — 슬롯이 하나라 롤백이 없습니다. CRC 는 무결성만 확인하고 서명 검증은 없습니다. 청크 재전송이 없어 실패하면 처음부터 다시 보내고, 평소 부팅도 3초 대기를 거칩니다.

---

## 장애 분석 — 조향 서보 소손 (2026-05 · 원인 특정 2026-07)

F446RE 이식은 주행까지 확인했습니다. 이틀 뒤 조종 반응 지연(큐에 쌓인 명령)을 고치던 커밋에서 **모터와 서보의 출력 타이머 매핑이 뒤바뀌었습니다.** 배선은 그대로라 서보가 스로틀 값을 위치 명령(0~999µs · 유효 범위 650~2350µs 밖)으로 받아 스톨했고, 여러 번 꺾여 속가닥이 끊어져 있던 GND 점퍼선이 그 전류를 견디지 못해 탔습니다. 같은 편집에서 비상정지도 새 매핑을 따라 뒤집혀 모터 드라이버를 끄지 못했습니다(공통원인고장). 망가진 것은 서보이고, L298N 은 이후 벤치 시험에서 정상으로 확인했습니다.

당시에는 하드웨어 문제로 남겨두었고, 약 3개월 뒤 커밋 이력을 대조해 원인을 특정했습니다. `firmware/F446RE` 코드는 사고 당시 상태 그대로 두었습니다 — 주석의 타이머 매핑이 서로 다르게 적혀 있는 것은 그 흔적입니다.

**재발 방지 원칙**
- 핀 ↔ 장치 매핑은 한 곳에서만 정의합니다
- 출력 경로를 바꾼 커밋은 액추에이터를 떼고 파형부터 확인합니다
- 한 커밋에 성격이 다른 변경을 섞지 않습니다
- 자동 생성 파일(`tim.c` 등)도 커밋합니다 — 이번엔 빠져 있어 사후 분석을 `.ioc` 에 기댔습니다
- 이상한 소리나 열이 나면 바로 전원을 끊습니다 — 이번엔 서보가 울리는데도 끊지 않았습니다

하드웨어 쪽 사후 분석은 [multi-mcu-can/docs/lesson_learned.md](https://github.com/steppenhj/multi-mcu-can/blob/main/docs/lesson_learned.md) 에 있습니다.

---

## 설계 다이어그램 (MBSE)

IBM Rhapsody · StarUML 로 그렸습니다.

<details>
<summary><b>유스케이스 · 클래스 · 시퀀스 · 상태차트</b></summary>

<br>

**유스케이스** — 조작자의 주행 제어가 코너링 부스트 · 출력 제한 · Return-to-Home 을 포함하고, 하드웨어 환경이 페일세이프에 참여합니다.

![Use Case Diagram](assets/usecase_diagram.png)

**클래스 (C++ 제어 코어)** — `SharedContext` 가 `std::mutex` · `std::atomic` 으로 공유 상태를 지키고, `UdpReceiver` 와 `VehicleController` 가 이를 참조합니다.

![Class Diagram](assets/architecture_diagram.png)

**시퀀스** — 조이스틱 입력이 Web UI → Python → C++ → STM32 로 가며 WebSocket JSON → UDP 바이너리 → UART 문자열로 바뀝니다.

![Sequence Diagram](assets/sequence_diagram.png)

**상태차트 (RTH · Fail-Safe)** — `OPERATING` → `RTH_RECORDING` → `RTH_ACTIVE`, `timeout == true` 이면 `FAIL_SAFE`.

![State Chart Diagram](assets/statechart_diagram.png)

</details>

<details>
<summary><b>Phase 6 CAN 초기 설계 (3노드)</b> — 구현은 multi-mcu-can 에서 2노드로</summary>

<br>

RPi5(Gateway) · F446RE(MotorECU) · F411RE(SensorECU) 3노드로 설계했습니다. RPi5 쪽은 MCP2515 의 5V 로직을 3.3V GPIO 에 바로 물릴 수 없어 접었고, 구현은 F446RE(bxCAN + SN65HVD230)와 F411RE(MCP2515 + TJA1050) 2노드로 [multi-mcu-can](https://github.com/steppenhj/multi-mcu-can) 에서 했습니다.

![Phase 6 Block Diagram](assets/phase6_block_diagram.png)

설계한 메시지 — `0x100 MotorCMD`(50ms) · `0x200 MotorStatus`(100ms) · `0x300 SensorData`(100ms). SensorECU 가 20cm 미만을 감지하면 MotorECU 가 스스로 멈추는 흐름입니다.

![Phase 6 Sequence Diagram](assets/phase6_sequence_diagram.png)

초기 설계안의 F446RE CAN 배선입니다. 직접 만든 웹 배선도 편집기로 그렸습니다([`tools/CAN_F446RE.json`](tools/CAN_F446RE.json)).

![F446RE CAN Wiring](tools/CAN_F446RE.png)

</details>

---

## 하드웨어 · 기술 스택

| 구성 | 내용 |
|---|---|
| MPU | Raspberry Pi 5 (4GB) |
| MCU | STM32 NUCLEO-F411RE (Phase 2~5) · NUCLEO-F446RE (Phase 6 이식) |
| 구동 | Ackermann 섀시 (엔코더 DC 모터 2 · 조향 서보) · L298N · Waveshare I2C Motor Driver HAT |
| 센서 | 엔코더 · HC-SR04P 초음파 |
| 전원 | LiPo 7.4V 2S + UBEC 5A |
| Web | HTML/CSS/JS · nipplejs · Socket.IO |
| 서버 | Python 3.11 · Flask-SocketIO · eventlet |
| 제어 코어 | C++17 · UDP · threads · mutex/atomic |
| 펌웨어 | C (STM32 HAL) · FreeRTOS · 커스텀 부트로더 |
| 설계 | IBM Rhapsody · StarUML · STM32CubeIDE |

전체 부품은 [docs/hardware.md](docs/hardware.md), 펌웨어 빌드는 [firmware/README.md](firmware/README.md) 에 있습니다.

<img src="assets/car_picture.jpg" alt="RPi 5 + STM32 Nucleo 제어 스택과 배선" width="400">

---

## 실행

```bash
# 1. 부트로더 — STM32CubeIDE 로 빌드해 ST-Link 로 굽습니다 (Sector 0)

# 2. 앱 — STM32CubeIDE 로 빌드한 .bin 을 OTA 로 올립니다
#    앱을 ST-Link 로 직접 구우면 유효 표식이 없어 부트로더가 실행하지 않습니다
cd web/
python3 ota_flasher.py <app.bin> /dev/ttyACM0
#    (웹 UI 에서 .bin 을 올려도 같은 과정을 거칩니다)

# 3. RPi 에서 C++ 제어 코어 빌드·실행 (저장소 루트)
g++ -std=c++17 -O2 -pthread -o drive_server src/control_core_oop.cpp
./drive_server

# 4. 다른 터미널에서 웹 서버
cd web/
python3 app.py
# 브라우저 → http://<rpi-ip>:5000
```

---

## 작성자

**박해진 (Haejin Park)**
