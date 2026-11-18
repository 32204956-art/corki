"""TI mmWave UART reader and Pi-to-PC radar state publisher."""
import argparse
import json
import math
import secrets
import socket
import struct
import threading
import time
from pathlib import Path

MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'
HEADER = struct.Struct('<8s8I')
POINT = struct.Struct('<4f')
TLV = struct.Struct('<II')
MAX_FRAME = 65536
FRAME_TTL_S = .25
PUBLISH_INTERVAL_S = .05


def decode_frame(packet):
    magic, _, size, _, frame_number, _, count, tlvs, subframe = HEADER.unpack_from(packet)
    if magic != MAGIC or size != len(packet) or tlvs > (size - HEADER.size) // TLV.size:
        raise ValueError('Invalid radar frame header')
    offset = HEADER.size
    points = None
    for _ in range(tlvs):
        kind, length = TLV.unpack_from(packet, offset)
        offset += TLV.size
        end = offset + length
        if end > size:
            raise ValueError('Truncated radar TLV')
        if kind == 1:
            if points is not None or length != count * POINT.size:
                raise ValueError('Unexpected detected-points payload')
            points = []
            for x, y, z, velocity in POINT.iter_unpack(packet[offset:end]):
                if not all(math.isfinite(v) for v in (x, y, z, velocity)):
                    raise ValueError('Invalid radar point')
                points.append(math.hypot(x, y, z))
        offset = end
    if points is None and count:
        raise ValueError('Missing detected-points TLV')
    if size - offset > 31:
        raise ValueError('Unexpected frame padding')
    return (frame_number, subframe), points or []


class FrameStream:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        while self.buffer:
            index = self.buffer.find(MAGIC)
            if index < 0:
                if len(self.buffer) > len(MAGIC)-1:
                    del self.buffer[:-(len(MAGIC)-1)]
                    yield None
                return
            if index:
                del self.buffer[:index]
                yield None
            if len(self.buffer) < HEADER.size:
                return
            length = struct.unpack_from('<I', self.buffer, 12)[0]
            if not HEADER.size <= length <= MAX_FRAME:
                del self.buffer[0]
                yield None
                continue
            if len(self.buffer) < length:
                return
            packet = bytes(self.buffer[:length])
            del self.buffer[:length]
            try:
                frame = decode_frame(packet)
            except (ValueError, struct.error):
                frame = None
            yield frame


class HazardDetector:
    def __init__(self):
        self.lock = threading.Lock()
        self.changed = threading.Event()
        self.state = 'UNKNOWN'
        self.hazard_epoch = 0
        self.received_at = None
        self.last_frame = None
        self.danger_streak = self.clear_streak = 0
        self.minimum = None
        self.danger_points = 0

    def _set_state(self, state):
        if state != self.state:
            self.state = state
            if state != 'CLEAR':
                self.hazard_epoch += 1
            self.changed.set()

    def _invalidate(self):
        self.received_at = None
        # Keep the frame ID so a replay cannot restore freshness after an error.
        self.danger_streak = self.clear_streak = 0
        self.minimum = None
        self.danger_points = 0
        self._set_state('UNKNOWN')

    def observe(self, frame, now):
        with self.lock:
            if frame is None:
                self._invalidate()
                return
            frame_id, distances = frame
            if self.last_frame is not None:
                old_number, old_subframe = self.last_frame
                delta = (frame_id[0] - old_number) & 0xffffffff
                if (delta == 0 and frame_id[1] <= old_subframe) or delta >= 0x80000000:
                    return
            if self.received_at is not None and now - self.received_at >= FRAME_TTL_S:
                self._invalidate()
            self.last_frame, self.received_at = frame_id, now
            self.minimum = min(distances, default=None)
            self.danger_points = sum(.15 <= d <= .40 for d in distances)
            self.danger_streak = self.danger_streak + 1 if self.danger_points >= 3 else 0
            self.clear_streak = self.clear_streak + 1 if not self.danger_points else 0
            if self.danger_streak >= 2:
                self._set_state('EVADE')
            elif self.clear_streak >= 2:
                self._set_state('CLEAR')

    def snapshot(self, now):
        with self.lock:
            if self.received_at is None or now - self.received_at >= FRAME_TTL_S:
                self._invalidate()
            return dict(state=self.state, sensor_valid=self.state != 'UNKNOWN', hazard_epoch=self.hazard_epoch,
                        min_distance_m=self.minimum, danger_points=self.danger_points)


def configure_sensor(serial_port, profile):
    commands = [line.strip() for line in Path(profile).read_text(encoding='utf-8-sig').splitlines()
                if line.strip() and not line.lstrip().startswith('%')]
    if not commands or commands[-1] != 'sensorStart':
        raise ValueError('Profile must finish with sensorStart')
    serial_port.reset_input_buffer()
    for command in commands:
        serial_port.write((command + '\n').encode('ascii'))
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            reply = serial_port.readline().strip()
            if b'error' in reply.lower():
                raise ValueError('Radar CLI rejected a configuration command')
            if reply == b'Done':
                break
        else:
            raise TimeoutError('Radar CLI did not acknowledge configuration')


def read_uart(args, detector, done, open_serial):
    while not done.is_set():
        try:
            with open_serial(args.data_port, 921600, timeout=.02) as data:
                data.reset_input_buffer()
                with open_serial(args.cli_port, 115200, timeout=.15, write_timeout=.5) as cli:
                    configure_sensor(cli, args.profile)
                frames = FrameStream()
                with detector.lock:
                    detector.last_frame = None
                while not done.is_set():
                    for frame in frames.feed(data.read(min(4096, data.in_waiting or 1))):
                        detector.observe(frame, time.monotonic())
        except Exception as exc:
            detector.observe(None, time.monotonic())
            print(f'Radar UART unavailable ({type(exc).__name__}): {exc}', flush=True)
            done.wait(.5)


def main():
    import ipaddress
    import serial

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pc', required=True, type=ipaddress.IPv4Address)
    parser.add_argument('--port', type=int, default=8891)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--cli-port', default='/dev/ttyUSB0')
    parser.add_argument('--data-port', default='/dev/ttyUSB1')
    args = parser.parse_args()
    if args.port != 8891:
        parser.error('Radar status uses PC UDP 8891')
    if args.pc.is_unspecified or args.pc.is_multicast or int(args.pc) == 0xffffffff:
        parser.error('Use the PC unicast IPv4 address')
    detector, done = HazardDetector(), threading.Event()
    threading.Thread(target=read_uart, args=(args, detector, done, serial.Serial), daemon=True).start()
    boot_id = secrets.randbelow(0xffffffff) + 1
    seq = 0
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
            while True:
                detector.changed.clear()
                seq += 1
                payload = dict(type='radar_status', protocol_version=1, boot_id=boot_id,
                               seq=seq, **detector.snapshot(time.monotonic()))
                try:
                    tx.sendto(json.dumps(payload, allow_nan=False).encode(), (str(args.pc), args.port))
                except OSError as exc:
                    print(f'Radar link error: {exc}', flush=True)
                detector.changed.wait(PUBLISH_INTERVAL_S)
    except KeyboardInterrupt:
        pass
    finally:
        done.set()


if __name__ == '__main__':
    main()
