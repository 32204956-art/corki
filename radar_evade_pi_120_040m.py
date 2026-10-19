import csv
import math
import struct
import time
from datetime import datetime
import serial

CLI_PORT = "/dev/ttyUSB0"
CLI_BAUD = 115200
DATA_PORT = "/dev/ttyUSB1"
DATA_BAUD = 921600

CFG_FILE = "profile_3d_aop_evade_040m_fov30.cfg"

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40
TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7

MIN_RANGE_M = 0.15
DANGER_DISTANCE_M = 0.40
APPROACH_DOPPLER_MPS = 1.20
MIN_DANGER_POINTS = 3
TRIGGER_FRAMES = 2

def send_cfg(cli, cfg_file):
    with open(cfg_file, "r", encoding="utf-8") as f:
        commands = []
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("%"):
                continue
            commands.append(line)

    cli.reset_input_buffer()

    for command in commands:
        cli.write((command + "\n").encode("ascii"))
        cli.flush()
        time.sleep(0.05)

        response = cli.read_all().decode(errors="ignore").strip()
        print(f"> {command}")
        if response:
            print(response)

        if "error" in response.lower():
            raise RuntimeError(f"Radar CLI error: {command}\n{response}")

def parse_frame(packet):
    if len(packet) < HEADER_SIZE:
        return None

    header = struct.unpack_from("<8s8I", packet, 0)
    magic = header[0]
    total_packet_len = header[2]
    frame_number = header[4]
    num_detected_obj = header[6]
    num_tlvs = header[7]

    if magic != MAGIC_WORD or total_packet_len > len(packet):
        return None

    offset = HEADER_SIZE
    points = []
    side_info = []

    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            break

        tlv_type, tlv_length = struct.unpack_from("<II", packet, offset)
        offset += 8
        payload_end = offset + tlv_length

        if payload_end > len(packet):
            break

        if tlv_type == TLV_DETECTED_POINTS:
            point_count = min(num_detected_obj, tlv_length // 16)
            for i in range(point_count):
                base = offset + i * 16
                x, y, z, doppler = struct.unpack_from("<ffff", packet, base)
                points.append({
                    "x": x, "y": y, "z": z, "doppler": doppler,
                    "snr": None, "noise": None
                })

        elif tlv_type == TLV_SIDE_INFO:
            side_count = min(num_detected_obj, tlv_length // 4)
            for i in range(side_count):
                base = offset + i * 4
                snr_raw, noise_raw = struct.unpack_from("<hh", packet, base)
                side_info.append({
                    "snr": snr_raw / 10.0,
                    "noise": noise_raw / 10.0
                })

        offset = payload_end

    for i in range(min(len(points), len(side_info))):
        points[i]["snr"] = side_info[i]["snr"]
        points[i]["noise"] = side_info[i]["noise"]

    return {"frame_number": frame_number, "points": points}

def is_danger_point(point):
    distance = math.sqrt(
        point["x"]**2 + point["y"]**2 + point["z"]**2
    )

    in_range = MIN_RANGE_M <= distance <= DANGER_DISTANCE_M
    approaching = point["doppler"] <= -APPROACH_DOPPLER_MPS

    return in_range and approaching, distance

def main():
    print("IWR6843AOP Raspberry Pi EVADE TEST")
    print(f"CLI  : {CLI_PORT} @ {CLI_BAUD}")
    print(f"DATA : {DATA_PORT} @ {DATA_BAUD}")
    print(f"Range: {MIN_RANGE_M:.2f} ~ {DANGER_DISTANCE_M:.2f} m")
    print(f"Doppler: <= -{APPROACH_DOPPLER_MPS:.2f} m/s")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_name = f"radar_evade_pi_{timestamp}.csv"

    data_serial = serial.Serial(DATA_PORT, DATA_BAUD, timeout=0.05)

    try:
        data_serial.reset_input_buffer()

        with serial.Serial(CLI_PORT, CLI_BAUD, timeout=0.2) as cli:
            time.sleep(0.5)
            send_cfg(cli, CFG_FILE)

        print("\n[READY] Radar data receiving...")
        print("[INFO] Ctrl+C to stop.\n")

        buffer = bytearray()
        danger_streak = 0

        with open(csv_name, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "timestamp_s", "frame_id", "point_id",
                "x_m", "y_m", "z_m", "range_m",
                "doppler_mps", "snr_db", "noise_db",
                "danger_point", "danger_point_count",
                "danger_streak", "state"
            ])

            while True:
                waiting = data_serial.in_waiting
                chunk = data_serial.read(waiting if waiting > 0 else 1)

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

                    if packet_len < HEADER_SIZE or packet_len > 65536:
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

                        processed.append(
                            (point_id, point, distance, danger)
                        )

                    if danger_count >= MIN_DANGER_POINTS:
                        danger_streak += 1
                    else:
                        danger_streak = 0

                    state = (
                        "EVADE"
                        if danger_streak >= TRIGGER_FRAMES
                        else "MONITOR"
                    )

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
        if data_serial.is_open:
            data_serial.close()
        print("[DONE] Serial port closed.")

if __name__ == "__main__":
    main()
