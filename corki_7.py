pkill -9 -f python3
cat << 'EOF' > corki_radar_irq.py
import math
import struct
import time
import socket
import json
import threading
import serial

# ============================================================
# 1. NETWORK & INTERRUPT CONFIG
# ============================================================
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 8890           # 조원 노트북 카메라 제스처 수신 포트
ESP32_IP = "10.154.181.136"  # ESP32 실제 IP
ESP32_PORT = 8888

CLI_PORT, CLI_BAUD = "/dev/ttyUSB0", 115200
DATA_PORT, DATA_BAUD = "/dev/ttyUSB1", 921600
CFG_FILE = "profile_3d_aop_evade_040m_fov30.cfg"

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40
TLV_DETECTED_POINTS = 1

MIN_RANGE_M = 0.15
WALL_DISTANCE_M = 0.40       # 벽 감지 임계 거리 (40cm)
MIN_DANGER_POINTS = 3
TRIGGER_FRAMES = 2
FORWARD_ESCAPE_MIN_VX = 0.5  # 대기 상태에서 허용할 최소 전진 속도 (cm/s)

# ============================================================
# 2. SHARED INTERRUPT STATE
# ============================================================
irq_wall_active = threading.Event()   # 현재 벽이 40cm 이내에 있음
irq_forward_latch = threading.Event() # 인터럽트 걸린 후 '전진만 허용' 대기 래치
state_lock = threading.Lock()

esp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_seq = 0


def send_esp32_immediate(status, vx=0.0, vy=0.0, w=0.0):
    """스레드 안전 즉시 UDP 전송 (인터럽트 발생 시 딜레이 없이 즉각 호출)"""
    global udp_seq
    with state_lock:
        udp_seq += 1
        seq = udp_seq

    if status == "STOP":
        vx, vy, w = 0.0, 0.0, 0.0

    payload = {
        "seq": seq,
        "t": float(time.monotonic()),
        "vx": float(vx),
        "vy": float(vy),
        "w": float(w),
        "status": status,
    }
    esp_sock.sendto(json.dumps(payload).encode("utf-8"), (ESP32_IP, ESP32_PORT))


def send_cfg(cli, cfg_file):
    with open(cfg_file, "r", encoding="utf-8") as f:
        commands = [r.strip() for r in f if r.strip() and not r.strip().startswith("%")]
    cli.reset_input_buffer()
    for cmd in commands:
        cli.write((cmd + "\n").encode("ascii"))
        cli.flush()
        time.sleep(0.05)


def parse_frame(packet):
    if len(packet) < HEADER_SIZE:
        return None
    header = struct.unpack_from("<8s8I", packet, 0)
    if header[0] != MAGIC_WORD or header[2] > len(packet):
        return None
    offset = HEADER_SIZE
    points = []
    for _ in range(header[7]):
        if offset + 8 > len(packet):
            break
        tlv_type, tlv_len = struct.unpack_from("<II", packet, offset)
        offset += 8
        if offset + tlv_len > len(packet):
            break
        if tlv_type == TLV_DETECTED_POINTS:
            for i in range(min(header[6], tlv_len // 16)):
                x, y, z, _ = struct.unpack_from("<ffff", packet, offset + i * 16)
                points.append((x, y, z))
        offset += tlv_len
    return points


# ============================================================
# 3. RADAR INTERRUPT SERVICE ROUTINE (ISR THREAD)
# ============================================================
def radar_irq_worker():
    while True:
        data_serial = None
        try:
            data_serial = serial.Serial(DATA_PORT, DATA_BAUD, timeout=0.01)
            data_serial.reset_input_buffer()
            with serial.Serial(CLI_PORT, CLI_BAUD, timeout=0.2) as cli:
                time.sleep(0.5)
                send_cfg(cli, CFG_FILE)
            print("[IRQ READY] Radar Interrupt Service Routine active.")

            buf = bytearray()
            streak = 0

            while True:
                waiting = data_serial.in_waiting
                chunk = data_serial.read(waiting if waiting > 0 else 1)
                if chunk:
                    buf.extend(chunk)

                while True:
                    idx = buf.find(MAGIC_WORD)
                    if idx < 0:
                        if len(buf) > 7:
                            del buf[:-7]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < 16:
                        break
                    plen = struct.unpack_from("<I", buf, 12)[0]
                    if plen < HEADER_SIZE or plen > 65536:
                        del buf[0]
                        continue
                    if len(buf) < plen:
                        break

                    pkt = bytes(buf[:plen])
                    del buf[:plen]
                    pts = parse_frame(pkt)
                    if pts is None:
                        continue

                    danger = sum(
                        1 for (x, y, z) in pts
                        if MIN_RANGE_M <= math.sqrt(x*x + y*y + z*z) <= WALL_DISTANCE_M
                    )
                    streak = (streak + 1) if danger >= MIN_DANGER_POINTS else 0

                    # [인터럽트 트리거] 벽 감지 순간 즉시 STOP 발사 + 전진 대기 래치 잠금
                    if streak >= TRIGGER_FRAMES:
                        if not irq_wall_active.is_set():
                            irq_wall_active.set()
                            irq_forward_latch.set()
                            send_esp32_immediate("STOP")
                            print("\n[IRQ TRIGGERED] Wall detected -> Immediate STOP & Forward-Only Wait!")
                    elif danger == 0 and irq_wall_active.is_set():
                        irq_wall_active.clear()
                        print("[IRQ CLEAR] Wall no longer in range (Still locked until forward cmd or clear)")

        except Exception as e:
            print(f"[RADAR/USB RECONNECT] {e}")
            time.sleep(0.5)
        finally:
            if data_serial and data_serial.is_open:
                data_serial.close()


# ============================================================
# 4. MAIN GATEWAY LOOP (CAMERA PASS-THROUGH WITH IRQ MASK)
# ============================================================
def main():
    # 초기 안전 정지 3회 전송
    for _ in range(3):
        send_esp32_immediate("STOP")
        time.sleep(0.05)

    # 레이더 인터럽트 스레드 시작
    t_radar = threading.Thread(target=radar_irq_worker, daemon=True)
    t_radar.start()

    cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cam_sock.bind((LISTEN_IP, LISTEN_PORT))
    cam_sock.setblocking(False)

    cam_cmd = {"status": "STOP", "vx": 0.0, "vy": 0.0, "w": 0.0}
    last_cam_rx = 0.0
    last_tx = 0.0
    last_mode = None

    print(f"[GATEWAY] Listening Camera UDP {LISTEN_PORT} -> ESP32 {ESP32_IP}:{ESP32_PORT}")

    try:
        while True:
            now = time.monotonic()

            # 카메라 패킷 수신
            while True:
                try:
                    raw, _ = cam_sock.recvfrom(1024)
                    msg = json.loads(raw.decode("utf-8"))
                    st = str(msg.get("status", "STOP")).upper()
                    cam_cmd = {
                        "status": st if st in ("RUN", "SLOW", "STOP") else "STOP",
                        "vx": float(msg.get("vx", 0.0)),
                        "vy": float(msg.get("vy", 0.0)),
                        "w": float(msg.get("w", 0.0)),
                    }
                    last_cam_rx = now
                except BlockingIOError:
                    break
                except Exception:
                    break

            cam_alive = (now - last_cam_rx) <= 0.5

            # 인터럽트 우선순위 판정
            if not cam_alive or cam_cmd["status"] == "STOP":
                out_st, out_vx, out_vy, out_w = "STOP", 0.0, 0.0, 0.0
                mode = "STOP_IDLE"
            elif irq_forward_latch.is_set():
                # 인터럽트 래치 상태: 후진/회전 전부 차단, 오직 전진(vx > 0.5)만 허용
                if cam_cmd["vx"] > FORWARD_ESCAPE_MIN_VX:
                    out_st = cam_cmd["status"]
                    out_vx = cam_cmd["vx"]
                    out_vy = 0.0
                    out_w = 0.0
                    mode = "IRQ_FORWARD_ESCAPE"
                    # 벽 범위 밖으로 빠져나왔다면 래치 완전 해제
                    if not irq_wall_active.is_set():
                        irq_forward_latch.clear()
                        print("[LATCH RELEASED] Normal 3-axis control restored.")
                else:
                    out_st, out_vx, out_vy, out_w = "STOP", 0.0, 0.0, 0.0
                    mode = "IRQ_WAIT_FORWARD_ONLY"
            else:
                out_st, out_vx, out_vy, out_w = cam_cmd["status"], cam_cmd["vx"], cam_cmd["vy"], cam_cmd["w"]
                mode = "NORMAL_PASS"

            if mode != last_mode:
                print(f"[MODE] {mode} | status={out_st} vx={out_vx:+.1f} vy={out_vy:+.1f} w={out_w:+.2f}")
                last_mode = mode

            # 20Hz(50ms) 주기로 ESP32 워치독 유지 전송
            if (now - last_tx) >= 0.05:
                send_esp32_immediate(out_st, out_vx, out_vy, out_w)
                last_tx = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[EXIT] Stopping motors...")
    finally:
        for _ in range(3):
            send_esp32_immediate("STOP")
            time.sleep(0.05)
        cam_sock.close()
        esp_sock.close()


if __name__ == "__main__":
    main()
EOF
python3 corki_radar_irq.py