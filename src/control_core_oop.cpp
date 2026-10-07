/**
 * @file control_core_oop.cpp
 * @brief 객체지향 리팩토링 (v2)
 * 
 * @details 아키텍처 개요:
 *  SharedContext   : 스레드 간 데이터 교환 (공용 칠판)
 *  UdpReceiver     : 네트워크 수신 담당 (Python app.py -> C++)
 *  serialParser    : UART 링버퍼 + 줄 단위 파싱 (STM32 -> RPi)
 *  RTHController   : Return-To-Home 경로 기록/복귀 로직
 *  VehicleContoller: 메인 제어 루프 (오케스트레이터 느낌)
 * 
 * @note 변경 이력:
 *      v1: 단일 controlLoop()에 모든 로직 집중
 *      v2: 책임 분리 - 각 클래스가 하나의 역할만 수행
 *
 *      RTH를 CPP (RPi) 에 넣은 이유, STM32가 아닌: mcu sram은 매우 작기에 memory overflow가 날 수 있다고 판단함
 *                                              또, M4 칩이니깐 FPU(부동소수점 연산 유닛)의 덕이 있지 않겠나 라고 생각함
 * 
 *      7/21 추가
 *      안전 정책(fail-stop): 통신 두절(watchdog timeout) 시 무조건 정지. 
 *                  RTH는 조작자가 트리거하는 '감독 하 자율(supervised autonomy)' - 
 *                  복귀 중에는 RPi 안 서버가 keep-alive 를 보내는 동안만 진행하며, 
 *                  서버 프로세스가 죽으면 즉시 FAIL_SAFE 정지 (브라우저·Wi-Fi 단절은 이 층에서 못 잡는다)
 *
 *  compile 명령어: g++ -std=c++17 -O2 -Wall -Wextra -pthread -o /dev/null src/control_core_oop.cpp
 */

 #include <iostream>
 #include <thread>
 #include <mutex>
 #include <atomic>
 #include <chrono>
 #include <cstring>
 #include <cmath>
 #include <algorithm>
 #include <unistd.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <fcntl.h>
 #include <termios.h>
 #include <arpa/inet.h>
 #include <vector>
 #include <climits>

 using namespace std;

 //==============================================================
 // 1. SharedContext - 스레드 간 공유 자원
 // ================================================================
 // "공용 칠판" 역할. 누가 쓰고 누가 읽는지 명확하게 구분해야 한다
 //     쓰는 쪽: UdpReceiver (throttle, steering, rth_mode)
 //     읽는 쪽: VehicleController (contrlLoop에서 매 틱마다)
 struct SharedContext {
    //------ 조종 명령 (UdpReceiver가 씀, VehicleController가 읽음)------
    float throttle = 0.0f;
    float steering = 0.0f;
    std::mutex data_mutex; //throttle, steering 보호용

    // ----- 시스템 플래그 (atomic -> mutex까진 불필요) ------
    std::atomic<int64_t> last_rx_us{0};  //WatchDog: 마지막 수신 시간 (마이크로s)
    std::atomic<bool> keep_running{true}; //프로그램 종료 플래그

    //-------RTH 상태 (UdpReceiver가 mode를 씀, VehicleController가 읽음)----
    std::atomic<int> rth_mode{0};  //0=일반, 1=기록중, 2=복귀중

    //****************************
    // Phase 6 들어가기 전,
    // Ping Pong으로 
    // 시간을 측정하자 (전체 시스템의 핑퐁)
    //  */
    std::atomic<bool> ping_requested{false};

    // ------RTH 경로 데이터 (RTHController가 관리) ------
    // path: {엔코더 tick 수, 서보 각도(PWM us)} 쌍의 리스트
    std::vector<std::pair<int, int>> path;
    std::mutex path_mutex; //path 벡터 보호용

    //유틸리티: 현재 시각(us)
    static int64_t now_us() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }
 };


 //==========================================================================
 // 2. UdpReceiver - 네트워크 수신 전담
 // ==================================================================================
 // app.py가 보내는 {throttle, steering, mode} 패킷을 받아서
 // SharedContext에 기록하는 역할만 한다
 class UdpReceiver {
private:
    int sockfd;
    int port;
    SharedContext& ctx;
    std::thread receiver_thread; //이게 지역변수가 아니라 멤버변수임!

    //Python과 약속한 패킷 구조 (struct.pack('ffi', ...))
    // 크기: float float int = 12bytes
    struct Packet {
        float th;
        float st;
        int mode;
    } __attribute__((packed)); //패딩 방지(Python struct.pack과 정확히 대응)

    void receiveLoop() {
        cout << "[UdpReceiver] Listening on port " << port << endl;

        sockaddr_in cliaddr{};
        socklen_t len = sizeof(cliaddr);
        Packet pkt;

        while(ctx.keep_running){
            int n = recvfrom(sockfd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&cliaddr, &len);

            if(n==(int)sizeof(pkt)){
                // --- 입력 검증: 네트워크 입력은 신뢰하지 않는다 ---
                // NaN/Inf는 (int) 캐스팅 시 미정의 동작 -> 패킷 폐기
                // 워치독 갱신 전에 걸러낸다 (유효한 패킷만 링크 생존 증거로 인정)
                if(!std::isfinite(pkt.th) || !std::isfinite(pkt.st)) continue;

                // 모드 화이트리스트 (FSM에 정의된 상태만 허용)
                if(pkt.mode != 0 && pkt.mode != 1 && pkt.mode != 2 && pkt.mode != 99) continue;

                //범위 클램프: 조종값 계약은 [-1, 1]
                float th = std::clamp(pkt.th, -1.0f, 1.0f);
                float st = std::clamp(pkt.st, -1.0f, 1.0f);

                //조종 명령 업데이트
                {
                    std::lock_guard<std::mutex> lock(ctx.data_mutex);
                    ctx.throttle = th;
                    ctx.steering = st;
                }
                // WatchDog 시간 갱신 (atomic이라 mutex 필요 없다)
                ctx.last_rx_us.store(SharedContext::now_us(), std::memory_order_relaxed);

                //*************
                // Phase 6전 핑퐁
                // mode=99일 때 ping 플래그 설정
                if(pkt.mode == 99){
                    ctx.ping_requested.store(true, std::memory_order_relaxed);
                    continue; //rth_mode를 99로 덮어쓰지 않도록
                }

                // RTH 모드 업데이트
                ctx.rth_mode.store(pkt.mode, std::memory_order_relaxed);
            }
            // n이 sizeof(Packet)과 다르면 -> 손생된 패킷, 무시
        }
    }

public:
    UdpReceiver(int _port, SharedContext& _ctx) : port(_port), ctx(_ctx) {
        // 소켓 생성
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        sockaddr_in servaddr{};
        servaddr.sin_family     = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port   = htons(port);

        //포트 재사용(프로세스 재시작 시 bind 실패 방지)
        int opt = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // 바인딩
        if(bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0){
            perror("[FATAL] UDP Bind Failed");
            exit(EXIT_FAILURE);
        }

        // 수신 타임아웃 1초 (스레드 종료 플래그 확인 주기)
        struct timeval tv = {1, 0};
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    ~UdpReceiver() {
        if(receiver_thread.joinable()) receiver_thread.join();
        close(sockfd);
        cout << "[UdpReceiver] Closed." << endl;
    }

    void start() {
        receiver_thread = std::thread(&UdpReceiver::receiveLoop, this);
    }
};



//===============================================================================================
// 3. SerialParser - UART 링버퍼 + 줄 단위 파싱
//=====================================================================================================
//문제: read()는 바이트 스트림이라 "ENC:123\n"이 한번에 올 보장이 없다.
//      "ENC:1" + "23\n"으로 쪼개질 수도 있고,
//      "ENC:123\nENC:467\n" 으로 붙어올 수도 있다.
//해결: 링버퍼에 누적하고, '\n'을 찾아서 한 줄씩 꺼낸다.
//      (STM32 쪽 HAL_UART_RxCpltCallback과 동일한 패턴)
class SerialParser {
private:
    // 링버퍼 (원형 큐)
    // 왜 링버퍼인가? -> 데이터가 계속 들어오는데, 매번 memmove하면 느리니까
    //      head/tail 포인터만 옮기면 0(1)
    static const int BUF_SIZE = 256;
    char ring[BUF_SIZE];
    // 메시지(ENC:123\n) 같이 짧은 텍스트 줄임. 한 줄에 10바이트 안팎.
    // 256이면 20~30줄이 밀려도 버틸 여유가 됨.
    // 임베디드에서 링버퍼 크기는 보통 
    // "최대 메시지 길이 * 처리 지연 동안 쌓일 수 있는 개수"
    int head = 0;  // 다음 쓸 위치
    int tail = 0;  // 다음 읽을 위치

    // 링버퍼에 데이터 추가
    void pushBytes(const char* data, int len){
        for(int i=0; i<len; i++){
            ring[head] = data[i];
            head = (head + 1) % BUF_SIZE;

            // 오버플로우 시 가장 오래된 데이터 버림 (tail 전진)
            if(head == tail) {
                tail = (tail + 1) % BUF_SIZE;
            }
        }
    }

    // 링버퍼에서 '\n'까지 한 줄 추출
    // 반환: true면 line에 한 줄이 들어감, false면 아직 완성된 줄 없음
    bool popLine(char* line, int max_len){
        int pos = tail;
        // int count = 0;

        // '\n' 찾기
        while(pos != head){
            if(ring[pos] == '\n') {
                // tail ~ pos 직전까지가 한 줄
                int i = 0;
                while(tail != pos && i < max_len - 1){
                    char c = ring[tail];
                    tail = (tail+1) % BUF_SIZE;
                    if(c != '\r') { // \r제거
                        line[i++] = c;
                    }
                }
                line[i] = '\0';

                // 7/23 추가
                // 줄이 max_len보다 길어 잘렸다면, 남은 조각을 개행 직전까지 전부 버린다
                // (안 버리면 다음 호출에서 잔여물이 "유령 줄"로 반환됨)
                while(tail != pos){
                    tail = (tail + 1) % BUF_SIZE;
                }

                // '\n' 자체도 소비
                tail = (tail + 1) % BUF_SIZE;
                return true;
            }
            pos = (pos+1) % BUF_SIZE;
            // count++;
        }
        return false;  // 아직 '\n'이 안 옴
    }

public:
    //serial fd에서 읽어서 링버퍼에 적재
    // 반환: 읽은 바이트 수 (0이면 데이터 없음)
    int feedFrom(int fd){
        char tmp[64];
        int n = read(fd, tmp, sizeof(tmp) - 1);
        if(n > 0){
            pushBytes(tmp, n);
        }
        return n;
    }

    // 링버퍼에서 완성된 줄 하나 꺼내기
    // 반환: true + line에 문자열, false면 줄 없음
    bool getLine(char* line, int max_line) {
        return popLine(line, max_line);
    }
};


//=============================================================================
// 4. RTHController - Return To Home 경로 기록/복귀
// =================================================================================
// 단일 책임: RTH 관련 상태 관리와 모터 명령 생성
// controlLoop()에서 이 로직을 분리함으로써,
// RTH 알고리즘을 수정할 때 다른 코드를 건드릴 필요가 없어짐.
class RTHController {
private:
    SharedContext& ctx;

    // RTH 복귀 상태 (한 스탭씩 역추적)
    int current_target = 0;     // 현재 스텝의 목표 거리(encoder ticks)
    int current_angle  = 1500;  // 현재 스텝의 서보 각도
    int acc_enc        = 0;     // 현재 스텝에서 누적된 거리
    int direction      = 1;     // 복귀 방향 (+1 or -1) -> 이건 소프트웨어적으로 잡았음. 뒤로 가면 부호만 바꾸면 됐음.

public:
    RTHController(SharedContext& _ctx) : ctx(_ctx) {}

    // --- RTH 모드 1: 경로 기록 ---
    // 엔코더 값이 들어올 때마다 호출
    // enc_val: 이번 주기의 엔코더 변화량
    // steering: 현재 조향값 (0~1 범위)
    void recordStep(int enc_val, float steering) {
        if(abs(enc_val) <= 2) return;  // 노이즈 필터 (정지 상태 무시)

        int pwm_angle = 1500 + (int)(steering * 900.0f);

        std::lock_guard<std::mutex> lock(ctx.path_mutex);
        ctx.path.push_back({enc_val, pwm_angle});
    } 

    // --- RTH 모드 2: 복귀 실행 ---
    // 반환: {속도(PWM), 각도(us)} - 모터에 직접 보낼 명령
    //      {0, 1500}이면 복귀 완료 (정지)
    struct MotorCmd { int speed; int angle; };

    MotorCmd executeStep(int enc_val) {
        std::lock_guard<std::mutex> lock(ctx.path_mutex);

        // 현재 스텝 소진 -> 다음 스텝 로드
        if (current_target == 0) {
            if(ctx.path.empty()) {
                // 경로 전부 소진 -> 복귀 완료
                ctx.rth_mode.store(0, std::memory_order_relaxed);
                reset();
                return {0, 1500};
            }

            // 스택(LIFO)에서 꺼냄 -> 역순 복귀
            auto [target, angle] = ctx.path.back();
            ctx.path.pop_back();

            current_target = abs(target);
            current_angle = angle;
            acc_enc = 0;
            direction = (target > 0) ? 1 : -1;
        }

        // 누적 거리 갱신
        acc_enc += abs(enc_val);

        // 남은 거리에 비례해서 감속
        // 최솟값 500: STM32 PID(kp=0.5) 통과 후 250 PWM -> 모터 구동 보장
        int remaining = current_target - acc_enc;
        int speed = min(600, max(500, remaining * 3));
        int reverse_speed = direction * speed;

        cout << "[RTH] target:" << current_target 
        << " acc:" << acc_enc 
        << " path_left:" << ctx.path.size() << endl;


        // 목표 도달 -> 다음 틱에서 새 스텝 로드
        if (acc_enc >= current_target) {
            current_target = 0;
        }

        return {reverse_speed, current_angle};
    }

    // 상태 초기화 (복귀 완료 또는 취소 시)
    void reset() {
        current_target  = 0;
        current_angle   = 1500;
        acc_enc         = 0;
        direction       = 1;
    }

    // 경로와 스텝 상태 전체 폐기 (새 기록 시작 또는 복귀 이탈 시)
    // 취소 후 '재개'를 지원하지 않는 이유: 취소 중 수동 주행하면
    // dead-reckoning 기준 위치가 어긋나 남은 경로가 무의미해지기 때문
    void discardPath(){
        std::lock_guard<std::mutex> lock(ctx.path_mutex);
        ctx.path.clear();
        reset();
    }
};


//===============================================================================
// 5. VehicleController - 메인 제어 루프 (오케스트레이터)
//==========================================================================================
// controlLoop() 의 역할: "흐름 관리"만 한다.
// 실제 작업은 SerialParser, RTHController에게 위임.
class VehicleController {
private:
    int serial_fd = -1;
    string device_name;
    SharedContext& ctx;
    std::thread control_thread; //이것도 지역변수가 아니라 멤버변수!

    // 상수
    static const int WATCHDOG_MS = 500;

    // 하위 모듈
    SerialParser parser;
    RTHController rth;

    // 엔코더 피드백용 UDP 소켓 (-> app.py:5556)
    int feedback_sock = -1;
    sockaddr_in feedback_addr{};

    // ------ 시리얼 포트 열기 (내부용) -----------
    bool openSerial() {
        serial_fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd == -1) return false;

        termios options{};
        tcgetattr(serial_fd, &options);

        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        // 8N1 설정
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        options.c_cflag |= (CLOCAL | CREAD | CS8);

        // Raw 모드 (canonical, echo 끄기)
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        tcsetattr(serial_fd, TCSANOW, &options);

        return true;
    }

    // ------ WatchDog 체크 --------
    bool isTimeout() {
        int64_t age_us = SharedContext::now_us() - ctx.last_rx_us.load();
        return (age_us > (int64_t)WATCHDOG_MS * 1000);
    }

    //------- 현재 명령 읽기 (mutex 보호) ------------
    struct Command { float throttle; float steering; };

    Command readCommand() {
        std::lock_guard<std::mutex> lock(ctx.data_mutex);
        return {ctx.throttle, ctx.steering};
    }

    // ------ 일반 주행 명령 전송 -----------
    void sendDriveCommand(float throttle, float steering){
        int pwm_speed = (int)(throttle * 999.0f);
        int pwm_angle = 1500 + (int)(steering * 900.0f);

        char buf[64];
        int len = snprintf(buf, sizeof(buf), "%d,%d\n", pwm_speed, pwm_angle);

        if(serial_fd != -1){
            write(serial_fd, buf, len);
        }
    }

    // --------RTH 명령 전송 ----------------
    void sendMotorCommand(int speed, int angle){
        char buf[64];
        int len = snprintf(buf, sizeof(buf), "%d,%d\n", speed, angle);

        if(serial_fd != -1){
            write(serial_fd, buf, len);
        }
    }

    // --------엔코더 피드백 전송 (-> app.py) -----
    void sendFeedback(const char* raw_line){
        sendto(feedback_sock, raw_line, strlen(raw_line), 0, (struct sockaddr*)&feedback_addr, sizeof(feedback_addr));
    }

    // -------- 엔코더 값 파싱 -------------
    // "ENC:1234" -> 1234반환, 파싱 실패 시 INT_MIN 반환
    static const int INVALID_ENC = INT_MIN;

    int parseEncoder(const char* line){
        if(strncmp(line, "ENC:", 4) == 0) {
            return atoi(line+4);
        }
        return INVALID_ENC;
    }

    //=======================================
    // ** controlLoop - 핵심
    // =============================
    // 이 함수는 "무엇을 하는가" 만 보여준다
    // "어떻게 하는가" 는 각 메서드/클래스가 담당하도록
    void controlLoop() {
        cout << "[VehicleController] Loop Started (100Hz)" << endl;

        using clock = std::chrono::steady_clock;
        auto next_tick = clock::now();

        int prev_rth_mode = 0; //모드 전환 감지용

        while(ctx.keep_running) {
            // --- 주기 설정 (10ms = 100Hz) ---
            next_tick += std::chrono::milliseconds(10);

            // 1. WatchDog 체크
            bool timeout = isTimeout();

            // 2. 현재 명령 읽기
            auto cmd = readCommand();
            if (timeout) {
                cmd.throttle = 0.0f;
                cmd.steering = 0.0f;
            }

            // 3. 일반 주행 명령 (RTH 복귀 중이 아닐 때만.)
            int rth_mode = ctx.rth_mode.load();

            // 모드 전환 감지: 새 기록 시작(->1) 또는 복귀 이탈(2->)이면 경로 폐기
            if(rth_mode != prev_rth_mode){
                if(rth_mode == 1 || prev_rth_mode == 2){
                    rth.discardPath();
                }
                prev_rth_mode = rth_mode;
            }

            if(rth_mode != 2) {
                sendDriveCommand(cmd.throttle, cmd.steering);
            }
            // 7/21추가
            else if(timeout){
                // FAIL_SAFE: 복귀 중 링크 상실 -> 즉시 정지
                // (상태차트의 RTH_ACTIVE -> FAIL_SAFE 전이 구현)
                // 5b와 달리 엔코더 수신에 의존하지 않고 매 틱 무조건 전송된다
                sendMotorCommand(0, 1500);
            }

            //**************
            // Ping-Pong
            // Ping 전송 로직 */
            if(ctx.ping_requested.exchange(false) && serial_fd != -1){
                const char* ping = "PING\n";
                write(serial_fd, ping, 5);
                cout << "[핑퐁-디버깅] PING sent to STM32" << endl; //보내는지 확인
            }

            // 4. 시리얼 읽기 (링버퍼에 적재)
            parser.feedFrom(serial_fd);

            // 5. 완성된 줄이 있으면 처리
            char line[64];
            while(parser.getLine(line, sizeof(line))) {

                //Pong 응답 처리 (Ping 측정용)
                if(strncmp(line, "PONG", 4) == 0){
                    cout << "[핑퐁-디버깅] Rx: " << line << endl; // Pong 받는지

                    sendFeedback("PONG");
                    continue;
                }
                
                int enc_val = parseEncoder(line);
                if (enc_val == INVALID_ENC) continue;

                // 5a. RTH 기록 모드
                if(rth_mode == 1){
                    rth.recordStep(enc_val, cmd.steering);
                }    

                // 5b. RTH 복귀 모드
                // !timeout = 서버 keep-alive 가 살아 있을 때만 복귀 진행 (서버 프로세스 생존 감시 · 브라우저 단절은 못 잡음)
                // 링크 상실 시에는 위 3번 분기가 매 틱 정지 명령을 보낸다 (fail-stop)
                if(rth_mode == 2 && !timeout) {
                    auto motor = rth.executeStep(enc_val);

                    if(motor.speed == 0 && motor.angle == 1500) {
                        // 복귀 완료 -> 정지
                        sendMotorCommand(0, 1500);
                    }
                    else{
                        sendMotorCommand(motor.speed, motor.angle);
                    }
                }

                // 5c. 피드백 전송 (app.py -> 웹 UI에 엔코더 표시)
                sendFeedback(line);
            }

            // 6. 정밀 주기 대기
            std::this_thread::sleep_until(next_tick);
        }
    }

public:
    VehicleController(string dev, SharedContext& _ctx)
        : device_name(dev), ctx(_ctx), rth(_ctx)
    {
        // 피드백 소켓: 시리얼 성공 여부와 무관하게 항상 초기화
        feedback_sock = socket(AF_INET, SOCK_DGRAM, 0);
        feedback_addr.sin_family = AF_INET;
        feedback_addr.sin_port = htons(5556);
        inet_pton(AF_INET, "127.0.0.1", &feedback_addr.sin_addr);

        if(!openSerial()){
            cerr << "[Error] Failed to Open serial: " << dev << endl;
        }
        else{
            cout << "[VehicleController] Serial opened: " << dev << endl;
        }
    }

    ~VehicleController() {
        if(control_thread.joinable()) control_thread.join();
        if(serial_fd != -1) close(serial_fd);
        if(feedback_sock > 0) close(feedback_sock);
        cout << "[VehicleController] Closed." << endl;
    }

    void start() {
        control_thread = std::thread(&VehicleController::controlLoop, this);
    }
};

// ==================================================================================================
// 6. main - 조립과 시작
// ====================================================================================================
int main() {
    cout << "=== Neuro-Drive Control (OOP v2) ===" << endl;

    // 1. 공유 자원 생성
    SharedContext shared;
    shared.last_rx_us = SharedContext::now_us();

    // 2. 객체 생성 (의존성 주입)
    UdpReceiver receiver(5555, shared);
    VehicleController car("/dev/ttyACM0", shared);
    // 이렇게 둘 다 호출하는 순간 스레드 2개가 동시에 돌기 시작함
    // 스레드의 수명을 객체와 일치시키려고 멤버변수!

    // 3. 시스템 가동
    receiver.start();
    car.start();

    // 4. 종료 대기
    cout << "Press ENTER to stop..." << endl;
    cin.get();

    // 5. 종료 절차
    cout << "Stopping..." << endl;
    shared.keep_running = false;
    // 소멸자에서 join() + 자원 해제 자동 수행

    return 0;
}