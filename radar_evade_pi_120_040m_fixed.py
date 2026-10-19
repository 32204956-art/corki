import csv
import glob
import math
import os
import struct
import time
from datetime import datetime

import serial
from serial import SerialException

CLI_BAUD = 115200
DATA_BAUD = 921600
CFG_FILE = "profile_3d_aop_evade_040m_fov30_fixed.cfg"

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40
TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7

MIN_RANGE_M = 0.15
DANGER_DISTANCE_M = 0.40
APPROACH_DOPPLER_MPS = 1.20
MIN_DANGER_POINTS = 3
TRIGGER_FRAMES = 2

MAX_PACKET_LEN = 65536
MAX_TLVS = 32
RECONNECT_TIMEOUT_S = 30.0


def find_radar_ports():
    base = "/dev/serial/by-id"
    cli = sorted(glob.glob(os.path.join(base, "*CP2105*if00-port0")))
    data = sorted(glob.glob(os.path.join(base, "*CP2105*if01-port0")))
    if cli and data:
        return cli[0], data[0]
    if os.path.exists("/dev/ttyUSB0") and os.path.exists("/dev/ttyUSB1"):
        return "/dev/ttyUSB0", "/dev/ttyUSB1"
    raise FileNotFoundError("CP2105 CLI/DATA ports not found")


def wait_for_radar_ports(timeout_s=RECONNECT_TIMEOUT_S):
    start = time.time()
    while True:
        try:
            return find_radar_ports()
        except FileNotFoundError:
            if time.time() - start >= timeout_s:
                raise
            print("[USB] Waiting for radar ports...")
            time.sleep(1.0)


def read_cli_response(cli, wait_s=0.12):
    time.sleep(wait_s)
    return cli.read_all().decode(errors="ignore").strip()


def send_cfg():
    cli_port, _ = wait_for_radar_ports()
    print(f"[CLI] {cli_port} @ {CLI_BAUD}")
    print(f"[CFG] {CFG_FILE}")

    with open(CFG_FILE, "r", encoding="utf-8") as f:
        commands = [
            line.strip() for line in f
            if line.strip() and not line.lstrip().startswith("%")
        ]

    with serial.Serial(cli_port, CLI_BAUD, timeout=0.2, write_timeout=1.0) as cli:
        time.sleep(0.3)
        cli.reset_input_buffer()

        for command in commands:
            cli.write((command + "\n").encode("ascii"))
            cli.flush()
            response = read_cli_response(cli)
            print(f"> {command}")
            if response:
                print(response)

            low = response.lower()
            if "not recognized" in low or "error" in low or "exception" in low:
                raise RuntimeError(
                    f"Radar rejected CFG command: {command}\n{response}"
                )

    print("[CFG] Configuration complete.")


def open_data_port():
    _, data_port = wait_for_radar_ports()
    print(f"[DATA] {data_port} @ {DATA_BAUD}")
    ser = serial.Serial(data_port, DATA_BAUD, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def recover_radar():
    print("\n[RECOVER] USB serial connection lost.")
    print("[RECOVER] Waiting for ports...")
    wait_for_radar_ports()
    time.sleep(0.5)
    print("[RECOVER] Re-sending CFG...")
    send_cfg()
    time.sleep(0.3)
    print("[RECOVER] Reopening DATA port...")
    return open_data_port()


def parse_frame(packet):
    if len(packet) < HEADER_SIZE:
        return None

    try:
        header = struct.unpack_from("<8s8I", packet, 0)
    except struct.error:
        return None

    magic = header[0]
    total_packet_len = header[2]
    frame_number = header[4]
    num_detected_obj = header[6]
    num_tlvs = header[7]

    if magic != MAGIC_WORD:
        return None
    if total_packet_len != len(packet):
        return None
    if num_tlvs > MAX_TLVS:
        return None

    offset = HEADER_SIZE
    points = []
    side_info = []

    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            return None

        try:
            tlv_type, tlv_length = struct.unpack_from("<II", packet, offset)
        except struct.error:
            return None

        offset += 8
        payload_end = offset + tlv_length
        if payload_end > len(packet):
            return None

        if tlv_type == TLV_DETECTED_POINTS:
            if tlv_length % 16 != 0:
                return None
            point_count = min(num_detected_obj, tlv_length // 16)
            for i in range(point_count):
                base = offset + i * 16
                try:
                    x, y, z, doppler = struct.unpack_from("<ffff", packet, base)
                except struct.error:
                    return None
                points.append({
                    "x": x,
                    "y": y,
                    "z": z,
                    "doppler": doppler,
                    "snr": None,
                    "noise": None,
                })

        elif tlv_type == TLV_SIDE_INFO:
            if tlv_length % 4 != 0:
                return None
            side_count = min(num_detected_obj, tlv_length // 4)
            for i in range(side_count):
                base = offset + i * 4
                try:
                    snr_raw, noise_raw = struct.unpack_from("<hh", packet, base)
                except struct.error:
                    return None
                side_info.append({
                    "snr": snr_raw / 10.0,
                    "noise": noise_raw / 10.0,
                })

        offset = payload_end

    for i in range(min(len(points), len(side_info))):
        points[i]["snr"] = side_info[i]["snr"]
        points[i]["noise"] = side_info[i]["noise"]

    return {"frame_number": frame_number, "points": points}


def is_danger_point(point):
    distance = math.sqrt(
        point["x"] ** 2 + point["y"] ** 2 + point["z"] ** 2
    )
    in_range = MIN_RANGE_M <= distance <= DANGER_DISTANCE_M
    approaching = point["doppler"] <= -APPROACH_DOPPLER_MPS
    return in_range and approaching, distance


def main():
    print("=" * 68)
    print("IWR6843AOP Raspberry Pi EVADE TEST - FIXED")
    print(f"Range   : {MIN_RANGE_M:.2f} ~ {DANGER_DISTANCE_M:.2f} m")
    print(f"Doppler : <= -{APPROACH_DOPPLER_MPS:.2f} m/s")
    print(f"Trigger : >= {MIN_DANGER_POINTS} points, {TRIGGER_FRAMES} frames")
    print("=" * 68)

    cli_port, data_port = wait_for_radar_ports()
    print(f"[PORT] CLI  = {cli_port}")
    print(f"[PORT] DATA = {data_port}")

    send_cfg()
    time.sleep(0.3)
    data_serial = open_data_port()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_name = f"radar_evade_pi_{timestamp}.csv"
    buffer = bytearray()
    danger_streak = 0

    print(f"[CSV] {csv_name}")
    print("[READY] Radar data receiving...")
    print("[INFO] Ctrl+C to stop.\n")

    try:
        with open(csv_name, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "timestamp_s", "frame_id", "point_id",
                "x_m", "y_m", "z_m", "range_m",
                "doppler_mps", "snr_db", "noise_db",
                "danger_point", "danger_point_count",
                "danger_streak", "state",
            ])

            while True:
                try:
                    waiting = data_serial.in_waiting
                    chunk = data_serial.read(waiting if waiting > 0 else 1)
                except SerialException:
                    try:
                        data_serial.close()
                    except Exception:
                        pass
                    buffer.clear()
                    danger_streak = 0
                    data_serial = recover_radar()
                    continue

                if chunk:
                    buffer.extend(chunk)

                while True:
                    magic_index = buffer.find(MAGIC_WORD)
                    if magic_index < 0:
                        if len(buffer) > 7:
                            del buffer[:-7]
                        break

                    if magic_index > 0:
                        del buffer[:magic_index]

                    if len(buffer) < 16:
                        break

                    packet_len = struct.unpack_from("<I", buffer, 12)[0]
                    if packet_len < HEADER_SIZE or packet_len > MAX_PACKET_LEN:
                        del buffer[0]
                        continue
                    if len(buffer) < packet_len:
                        break

                    packet = bytes(buffer[:packet_len])
                    del buffer[:packet_len]

                    frame = parse_frame(packet)
                    if frame is None:
                        continue

                    processed = []
                    danger_count = 0

                    for point_id, point in enumerate(frame["points"]):
                        danger, distance = is_danger_point(point)
                        if danger:
                            danger_count += 1
                        processed.append((point_id, point, distance, danger))

                    if danger_count >= MIN_DANGER_POINTS:
                        danger_streak += 1
                    else:
                        danger_streak = 0

                    state = "EVADE" if danger_streak >= TRIGGER_FRAMES else "MONITOR"

                    if processed:
                        min_range = min(row[2] for row in processed)
                        min_doppler = min(row[1]["doppler"] for row in processed)
                        rtxt = f"{min_range:.3f}m"
                        vtxt = f"{min_doppler:+.3f}m/s"
                    else:
                        rtxt = "-"
                        vtxt = "-"

                    print(
                        f"Frame {frame['frame_number']:6d} | "
                        f"points={len(frame['points']):2d} | "
                        f"danger={danger_count:2d} | "
                        f"streak={danger_streak:2d} | "
                        f"minR={rtxt:>7} | "
                        f"minV={vtxt:>11} | "
                        f"{state}"
                    )

                    now = time.time()
                    for point_id, point, distance, danger in processed:
                        writer.writerow([
                            f"{now:.3f}",
                            frame["frame_number"],
                            point_id,
                            f"{point['x']:.5f}",
                            f"{point['y']:.5f}",
                            f"{point['z']:.5f}",
                            f"{distance:.5f}",
                            f"{point['doppler']:.5f}",
                            "" if point["snr"] is None else f"{point['snr']:.1f}",
                            "" if point["noise"] is None else f"{point['noise']:.1f}",
                            int(danger),
                            danger_count,
                            danger_streak,
                            state,
                        ])
                    csv_file.flush()

    except KeyboardInterrupt:
        print("\n[STOP] User requested stop.")
    finally:
        try:
            data_serial.close()
        except Exception:
            pass
        print("[DONE] Serial port closed.")


if __name__ == "__main__":
    main()
