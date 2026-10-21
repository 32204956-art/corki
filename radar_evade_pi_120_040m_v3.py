import csv
import math
import struct
import time
import socket
import json
from datetime import datetime

import serial

# ============================================================
# ESP32 UDP CONTROL
# ============================================================

ESP32_IP = "10.43.139.108"
ESP32_PORT = 8888

# 1차 회피 시험 속도
# EVADE 동안 왼쪽(+vy)으로 5 cm/s 이동
EVADE_VX_CM_S = 0.0
EVADE_VY_CM_S = 5.0
EVADE_W_RAD_S = 0.0

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_seq = 0


# ============================================================
# RADAR SERIAL
# ============================================================

CLI_PORT = "/dev/ttyUSB0"
CLI_BAUD = 115200

DATA_PORT = "/dev/ttyUSB1"
DATA_BAUD = 921600

CFG_FILE = "profile_3d_aop_evade_040m_fov30.cfg"

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40

TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7


# ============================================================
# EVADE CONDITION
# ============================================================

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

        response = cli.read_all().decode(
            errors="ignore"
        ).strip()

        print(f"> {command}")

        if response:
            print(response)

        if "error" in response.lower():
            raise RuntimeError(
                f"Radar CLI error: {command}\n{response}"
            )


def parse_frame(packet):
    if len(packet) < HEADER_SIZE:
        return None

    header = struct.unpack_from("<8s8I", packet, 0)

    magic = header[0]
    total_packet_len = header[2]
    frame_number = header[4]
    num_detected_obj = header[6]
    num_tlvs = header[7]

    if magic != MAGIC_WORD:
        return None

    if total_packet_len > len(packet):
        return None

    offset = HEADER_SIZE

    points = []
    side_info = []

    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            break

        tlv_type, tlv_length = struct.unpack_from(
            "<II",
            packet,
            offset
        )

        offset += 8

        payload_end = offset + tlv_length

        if payload_end > len(packet):
            break

        if tlv_type == TLV_DETECTED_POINTS:
            point_count = min(
                num_detected_obj,
                tlv_length // 16
            )

            for i in range(point_count):
                base = offset + i * 16

                x, y, z, doppler = struct.unpack_from(
                    "<ffff",
                    packet,
                    base
                )

                points.append({
                    "x": x,
                    "y": y,
                    "z": z,
                    "doppler": doppler,
                    "snr": None,
                    "noise": None
                })

        elif tlv_type == TLV_SIDE_INFO:
            side_count = min(
                num_detected_obj,
                tlv_length // 4
            )

            for i in range(side_count):
                base = offset + i * 4

                snr_raw, noise_raw = struct.unpack_from(
                    "<hh",
                    packet,
                    base
                )

                side_info.append({
                    "snr": snr_raw / 10.0,
                    "noise": noise_raw / 10.0
                })

        offset = payload_end

    for i in range(min(len(points), len(side_info))):
        points[i]["snr"] = side_info[i]["snr"]
        points[i]["noise"] = side_info[i]["noise"]

    return {
        "frame_number": frame_number,
        "points": points
    }


def is_danger_point(point):
    distance = math.sqrt(
        point["x"] ** 2
        + point["y"] ** 2
        + point["z"] ** 2
    )

    in_range = (
        MIN_RANGE_M
        <= distance
        <= DANGER_DISTANCE_M
    )

    approaching = (
        point["doppler"]
        <= -APPROACH_DOPPLER_MPS
    )

    return in_range and approaching, distance


def send_esp32_command(state):
    global udp_seq

    udp_seq += 1

    if state == "EVADE":
        status = "RUN"

        vx = EVADE_VX_CM_S
        vy = EVADE_VY_CM_S
        w = EVADE_W_RAD_S

    else:
        status = "STOP"

        vx = 0.0
        vy = 0.0
        w = 0.0

    command = {
        "seq": udp_seq,
        "t": float(time.monotonic()),
        "vx": float(vx),
        "vy": float(vy),
        "w": float(w),
        "status": status
    }

    message = json.dumps(command).encode("utf-8")

    udp_socket.sendto(
        message,
        (ESP32_IP, ESP32_PORT)
    )


def main():
    data_serial = None

    print("=" * 68)
    print("IWR6843AOP Raspberry Pi EVADE + ESP32 TEST")
    print(f"CLI     : {CLI_PORT} @ {CLI_BAUD}")
    print(f"DATA    : {DATA_PORT} @ {DATA_BAUD}")
    print(f"ESP32   : {ESP32_IP}:{ESP32_PORT}")
    print(
        f"Range   : "
        f"{MIN_RANGE_M:.2f} ~ "
        f"{DANGER_DISTANCE_M:.2f} m"
    )
    print(
        f"Doppler : "
        f"<= -{APPROACH_DOPPLER_MPS:.2f} m/s"
    )
    print(
        f"EVADE   : "
        f"vx={EVADE_VX_CM_S:.1f}, "
        f"vy={EVADE_VY_CM_S:.1f}, "
        f"w={EVADE_W_RAD_S:.1f}"
    )
    print("=" * 68)

    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    csv_name = (
        f"radar_evade_pi_{timestamp}.csv"
    )

    try:
        # ----------------------------------------------------
        # 시작할 때 ESP32를 우선 STOP 상태로 만든다.
        # ----------------------------------------------------
        send_esp32_command("MONITOR")
        time.sleep(0.35)
        send_esp32_command("MONITOR")

        print("[UDP] Initial STOP sent to ESP32.")

        # ----------------------------------------------------
        # Radar DATA port open
        # ----------------------------------------------------
        data_serial = serial.Serial(
            DATA_PORT,
            DATA_BAUD,
            timeout=0.05
        )

        data_serial.reset_input_buffer()

        # ----------------------------------------------------
        # Radar CFG send
        # ----------------------------------------------------
        with serial.Serial(
            CLI_PORT,
            CLI_BAUD,
            timeout=0.2
        ) as cli:

            time.sleep(0.5)

            send_cfg(
                cli,
                CFG_FILE
            )

        print()
        print("[READY] Radar data receiving...")
        print("[INFO] Ctrl+C to stop.")
        print()

        buffer = bytearray()
        danger_streak = 0

        with open(
            csv_name,
            "w",
            newline="",
            encoding="utf-8"
        ) as csv_file:

            writer = csv.writer(csv_file)

            writer.writerow([
                "timestamp_s",
                "frame_id",
                "point_id",
                "x_m",
                "y_m",
                "z_m",
                "range_m",
                "doppler_mps",
                "snr_db",
                "noise_db",
                "danger_point",
                "danger_point_count",
                "danger_streak",
                "state"
            ])

            while True:
                waiting = data_serial.in_waiting

                chunk = data_serial.read(
                    waiting
                    if waiting > 0
                    else 1
                )

                if chunk:
                    buffer.extend(chunk)

                while True:
                    magic_index = buffer.find(
                        MAGIC_WORD
                    )

                    if magic_index < 0:
                        if len(buffer) > 7:
                            del buffer[:-7]

                        break

                    if magic_index > 0:
                        del buffer[:magic_index]

                    if len(buffer) < 16:
                        break

                    packet_len = struct.unpack_from(
                        "<I",
                        buffer,
                        12
                    )[0]

                    if (
                        packet_len < HEADER_SIZE
                        or packet_len > 65536
                    ):
                        del buffer[0]
                        continue

                    if len(buffer) < packet_len:
                        break

                    packet = bytes(
                        buffer[:packet_len]
                    )

                    del buffer[:packet_len]

                    frame = parse_frame(packet)

                    if frame is None:
                        continue

                    processed = []
                    danger_count = 0

                    for point_id, point in enumerate(
                        frame["points"]
                    ):
                        danger, distance = (
                            is_danger_point(point)
                        )

                        if danger:
                            danger_count += 1

                        processed.append(
                            (
                                point_id,
                                point,
                                distance,
                                danger
                            )
                        )

                    if (
                        danger_count
                        >= MIN_DANGER_POINTS
                    ):
                        danger_streak += 1
                    else:
                        danger_streak = 0

                    state = (
                        "EVADE"
                        if danger_streak
                        >= TRIGGER_FRAMES
                        else "MONITOR"
                    )

                    # ========================================
                    # Raspberry Pi -> ESP32
                    #
                    # MONITOR -> STOP
                    # EVADE   -> RUN + 왼쪽 이동
                    # ========================================
                    send_esp32_command(state)

                    if processed:
                        min_range = min(
                            row[2]
                            for row in processed
                        )

                        min_doppler = min(
                            row[1]["doppler"]
                            for row in processed
                        )

                        rtxt = (
                            f"{min_range:.3f}m"
                        )

                        vtxt = (
                            f"{min_doppler:+.3f}m/s"
                        )

                    else:
                        rtxt = "-"
                        vtxt = "-"

                    print(
                        f"Frame "
                        f"{frame['frame_number']:6d} | "
                        f"points="
                        f"{len(frame['points']):2d} | "
                        f"danger="
                        f"{danger_count:2d} | "
                        f"streak="
                        f"{danger_streak:2d} | "
                        f"minR={rtxt:>7} | "
                        f"minV={vtxt:>11} | "
                        f"{state}"
                    )

                    now = time.time()

                    for (
                        point_id,
                        point,
                        distance,
                        danger
                    ) in processed:

                        writer.writerow([
                            f"{now:.3f}",
                            frame["frame_number"],
                            point_id,
                            f"{point['x']:.5f}",
                            f"{point['y']:.5f}",
                            f"{point['z']:.5f}",
                            f"{distance:.5f}",
                            f"{point['doppler']:.5f}",
                            (
                                ""
                                if point["snr"] is None
                                else f"{point['snr']:.1f}"
                            ),
                            (
                                ""
                                if point["noise"] is None
                                else f"{point['noise']:.1f}"
                            ),
                            int(danger),
                            danger_count,
                            danger_streak,
                            state
                        ])

                    csv_file.flush()

    except KeyboardInterrupt:
        print()
        print("[STOP] User requested stop.")

    finally:
        # ================================================
        # 프로그램 종료 시 ESP32에 마지막 STOP 전송
        # ================================================
        try:
            send_esp32_command("MONITOR")
            time.sleep(0.1)

            print(
                "[UDP] Final STOP sent to ESP32."
            )

        except Exception as e:
            print(
                f"[UDP] Final STOP send failed: {e}"
            )

        if (
            data_serial is not None
            and data_serial.is_open
        ):
            data_serial.close()

        udp_socket.close()

        print(
            "[DONE] Serial and UDP sockets closed."
        )


if __name__ == "__main__":
    main()