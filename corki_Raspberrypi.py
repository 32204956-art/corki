"""Rear radar stop interlock + camera-motion UDP gateway for Raspberry Pi 5.

The radar and camera must NOT both send to the ESP32's UDP control port.
Camera's gesture processor sends motion requests here on UDP port 8890;
this script is the only writer to ESP32 UDP port 8888.

Camera motion request examples:
    {"status":"STOP","vx":0,"vy":0,"w":0}
    {"status":"RUN","vx":5,"vy":0,"w":0}

After radar STOP, a camera STOP then a fresh RUN is needed to re-arm.
"""

import json
import math
import os
import socket
import struct
import time

import serial

# --- Project settings: edit these if addresses or wiring change ---
ESP32_IP = "10.154.181.136"  # Check Arduino Serial Monitor Wi-Fi IP
ESP32_PORT = 8888
CAMERA_UDP_PORT = 8890       # Gesture controller -> Raspberry Pi

CLI_PORT = "/dev/ttyUSB0"
DATA_PORT = "/dev/ttyUSB1"
CLI_BAUD = 115200
DATA_BAUD = 921600
CFG_FILE = "profile_3d_aop_rear_stop_040m.cfg"

# Radar ROI, in metres (static obstacles included; no Doppler criterion)
NEAR_M = 0.15
STOP_DISTANCE_M = 0.40
MIN_OBSTACLE_POINTS = 2        # Tune after bench testing: increase if noisy
DETECT_FRAMES = 2             # Consecutive frames with enough points
CLEAR_FRAMES = 5              # Consecutive clear frames
STOP_HOLD_S = 1.0             # Remain stopped after last detection
RADAR_DATA_TIMEOUT_S = 0.50  # Fail closed if radar stops producing frames
CAMERA_CMD_TIMEOUT_S = 0.40  # Fail closed if camera commands stop arriving
UDP_PERIOD_S = 0.10          # Keep ESP32 watchdog alive (300 ms)
MAX_VELOCITY_CM_S = 10.0
MAX_W_RAD_S = 1.0

MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40


def send_cfg():
    if not os.path.exists(CFG_FILE):
        raise FileNotFoundError(f"Missing radar CFG: {CFG_FILE}")
    with serial.Serial(CLI_PORT, CLI_BAUD, timeout=0.20) as cli:
        time.sleep(0.30)
        cli.reset_input_buffer()
        with open(CFG_FILE, encoding="utf-8") as f:
            for raw in f:
                cmd = raw.strip()
                if not cmd or cmd.startswith(("%", "#")):
                    continue
                cli.write((cmd + "\n").encode("ascii"))
                cli.flush()
                time.sleep(0.07)
                answer = cli.read_all().decode(errors="replace")
                if "not recognized" in answer.lower() or "error" in answer.lower():
                    raise RuntimeError(f"Radar rejected {cmd}: {answer.strip()}")
        print("[RADAR] Configuration loaded")


def parse_points(packet):
    """Parse mmWave SDK 3.x OOB point-cloud TLV (same layout as prior code).

    Returns (frame_number, points) or None if malformed. Empty list is a valid
    no-point frame. The first TLV is detected-points when objects are present.
    """
    if len(packet) < HEADER_SIZE or packet[:8] != MAGIC:
        return None
    header = struct.unpack_from("<8s8I", packet)
    total_len = header[2]
    frame_no = header[4]
    count = header[6]
    tlvs = header[7]
    if total_len != len(packet) or count > 2048 or tlvs > 50:
        return None
    if count == 0:
        return frame_no, []
    if not tlvs or len(packet) < HEADER_SIZE + 8:
        return None
    tlv_type, tlv_length = struct.unpack_from("<II", packet, HEADER_SIZE)
    if tlv_type != 1:
        return None

    # Different SDK demos may report payload length or length including TLV header.
    # For detected-points TLV, validate against the known 16-byte point structure.
    required = count * 16
    if tlv_length not in (required, required + 8):
        return None
    start = HEADER_SIZE + 8
    if start + required > len(packet):
        return None
    points = []
    for i in range(count):
        x, y, z, doppler = struct.unpack_from("<ffff", packet, start + i * 16)
        if all(map(math.isfinite, (x, y, z, doppler))):
            points.append((x, y, z))
    return frame_no, points


def obstacle_point_count(points):
    # Radar's existing cfg limits FoV to +/-30 deg azimuth and -10~20 elevation.
    # Distance here is the Euclidean distance from the sensor.
    return sum(
        1 for x, y, z in points
        if NEAR_M <= math.sqrt(x * x + y * y + z * z) <= STOP_DISTANCE_M
    )


def decode_camera_command(raw):
    try:
        data = json.loads(raw.decode("utf-8"))
        status = data["status"].upper()
        if status not in ("RUN", "STOP", "SLOW"):
            return None
        vx, vy, w = (float(data[k]) for k in ("vx", "vy", "w"))
        if not all(map(math.isfinite, (vx, vy, w))):
            return None
        if math.hypot(vx, vy) > MAX_VELOCITY_CM_S or abs(w) > MAX_W_RAD_S:
            return None
        if status == "STOP":
            return "STOP", 0.0, 0.0, 0.0
        return status, vx, vy, w
    except (ValueError, TypeError, KeyError, AttributeError, UnicodeError):
        return None


def main():
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera.bind(("0.0.0.0", CAMERA_UDP_PORT))
    camera.setblocking(False)
    radar = None
    seq = 0

    def send_motor(status="STOP", vx=0.0, vy=0.0, w=0.0):
        nonlocal seq
        seq += 1
        message = {
            "seq": seq, "t": float(time.monotonic()),
            "vx": float(vx), "vy": float(vy), "w": float(w),
            "status": status,
        }
        udp.sendto(json.dumps(message).encode(), (ESP32_IP, ESP32_PORT))

    try:
        # Send only STOP while radar is not yet ready.
        send_motor()
        time.sleep(0.35)
        send_motor()
        radar = serial.Serial(DATA_PORT, DATA_BAUD, timeout=0.02)
        radar.reset_input_buffer()
        send_cfg()

        print(f"[READY] ESP32 {ESP32_IP}:{ESP32_PORT}")
        print(f"[CAMERA] Send desired velocity to Pi UDP {CAMERA_UDP_PORT}")
        print("[RADAR] Rear obstacle guard active; no Doppler threshold")
        print("[SAFETY] After an obstacle, send camera STOP and a new RUN")

        buf = bytearray()
        good_frame_at = 0.0
        detect_streak = 0
        clear_streak = 0
        stop_until = 0.0
        blocked = True  # Locked until several healthy clear radar frames.
        radar_healthy = False

        camera_req = ("STOP", 0.0, 0.0, 0.0)
        camera_req_at = 0.0
        needs_new_gesture = True   # Require STOP then fresh RUN at startup
        camera_stop_seen = False
        last_tx = 0.0
        last_sent = None

        while True:
            now = time.monotonic()
            # Drain camera commands, keeping the most recent.
            while True:
                try:
                    packet, addr = camera.recvfrom(1024)
                except BlockingIOError:
                    break
                cmd = decode_camera_command(packet)
                if cmd is None:
                    continue
                camera_req = cmd
                camera_req_at = now
                if cmd[0] == "STOP":
                    camera_stop_seen = True
                elif (not blocked and radar_healthy and needs_new_gesture
                      and camera_stop_seen):
                    needs_new_gesture = False
                    print("[CAMERA] Fresh gesture accepted after radar stop")

            waiting = radar.in_waiting
            data = radar.read(min(waiting, 4096) if waiting else 1)
            if data:
                buf.extend(data)
                # Bound memory in case of serial corruption.
                if len(buf) > 131072:
                    del buf[:-8]

            while True:
                start = buf.find(MAGIC)
                if start == -1:
                    if len(buf) > 7:
                        del buf[:-7]
                    break
                if start:
                    del buf[:start]
                if len(buf) < 16:
                    break
                packet_len = struct.unpack_from("<I", buf, 12)[0]
                if packet_len < HEADER_SIZE or packet_len > 65536:
                    del buf[0]
                    continue
                if len(buf) < packet_len:
                    break
                result = parse_points(bytes(buf[:packet_len]))
                del buf[:packet_len]
                if result is None:
                    continue
                frame_no, points = result
                now = time.monotonic()
                good_frame_at = now
                radar_healthy = True
                count = obstacle_point_count(points)
                if count >= MIN_OBSTACLE_POINTS:
                    detect_streak += 1
                    clear_streak = 0
                    stop_until = now + STOP_HOLD_S
                    if detect_streak >= DETECT_FRAMES:
                        if not blocked:
                            print(f"[REAR BLOCKED] frame={frame_no}, points={count}")
                            needs_new_gesture = True
                            camera_stop_seen = False
                        blocked = True
                else:
                    detect_streak = 0
                    clear_streak += 1
                    if blocked and clear_streak >= CLEAR_FRAMES and now >= stop_until:
                        blocked = False
                        needs_new_gesture = True
                        camera_stop_seen = False
                        print(f"[REAR CLEAR] frame={frame_no}; waiting for fresh STOP -> RUN gesture")

            now = time.monotonic()
            if good_frame_at == 0 or now - good_frame_at > RADAR_DATA_TIMEOUT_S:
                if radar_healthy:
                    print("[RADAR LOST] No valid frames; STOP")
                radar_healthy = False
                blocked = True
                needs_new_gesture = True
                camera_stop_seen = False
                detect_streak = 0
                clear_streak = 0

            camera_fresh = now - camera_req_at <= CAMERA_CMD_TIMEOUT_S
            if (radar_healthy and not blocked and not needs_new_gesture
                    and camera_fresh and camera_req[0] != "STOP"):
                output = camera_req
            else:
                output = ("STOP", 0.0, 0.0, 0.0)

            if output != last_sent or now - last_tx >= UDP_PERIOD_S:
                send_motor(*output)
                if output[0] != (last_sent[0] if last_sent else None):
                    print(f"[MOTOR] {output[0]} (rear={'BLOCK' if blocked else 'CLEAR'})")
                last_sent = output
                last_tx = now

    except KeyboardInterrupt:
        print("\n[EXIT] User stopped")
    except (serial.SerialException, OSError) as e:
        print(f"[RADAR/USB ERROR] {e}; stopping motors")
    finally:
        for _ in range(3):
            try:
                send_motor()
                time.sleep(0.10)
            except OSError:
                pass
        if radar and radar.is_open:
            radar.close()
        camera.close()
        udp.close()
        print("[DONE] STOP sent, ports closed")


if __name__ == "__main__":
    main()
