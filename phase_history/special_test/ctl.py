# ctl.py — 노트북이 RPi 대신 보내는 최소 조종기
import serial, msvcrt, time
PORT = "COM5"            # 장치 관리자에서 STLink Virtual COM Port 번호
ser = serial.Serial(PORT, 115200, timeout=0)
speed, angle = 0, 1500
print("w/s 전후  a/d 조향  x 정지  q 종료")
try:
    while True:
        if msvcrt.kbhit():
            k = msvcrt.getch().decode(errors="ignore")
            if k == "w": speed = 700
            elif k == "s": speed = -700
            elif k == "a": angle = 1200
            elif k == "d": angle = 1800
            elif k == "x": speed, angle = 0, 1500
            elif k == "p": time.sleep(2) #WatchDog 실험
            elif k == "q": break
        ser.write(f"{speed},{angle}\n".encode())
        line = ser.readline().decode(errors="ignore").strip()
        if line: print(line)     # ENC:rpm, PONG 등 회신
        time.sleep(0.05)         # 20Hz — 워치독 500ms 안쪽
finally:
    ser.write(b"0,1500\n"); ser.close()
