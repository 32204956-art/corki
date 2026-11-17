"""Send *requests* from the existing vision controller to the Raspberry Pi gateway.

Not a direct ESP32 connection. UDP has NO delivery acknowledgement. The Pi gateway
is the only process allowed to write motor commands to ESP32:8888.

A separate thread sends STOP on stale frames or a paused/lost target.
The existing program's SIMULATION and MOTOR TEST behavior is unaffected.
"""

import ipaddress
import json
import math
import socket
import threading
import time


class PiGatewayClient:
    PERIOD_S = 0.10
    MAX_LINEAR_CM_S = 3.0       # Conservative bench-test cap
    MAX_ANGULAR_RAD_S = 0.30

    def __init__(self, host, port=8890, stale_s=0.25, clock=time.monotonic,
                 socket_factory=socket.socket):
        host = str(ipaddress.IPv4Address(host))
        if not 1 <= port <= 65535:
            raise ValueError('Invalid Pi UDP port')
        if not 0.05 <= stale_s <= 0.5:
            raise ValueError('Invalid perception stale timeout')
        self.dest = (host, port)
        self.clock = clock
        self.stale_s = stale_s
        self.socket_factory = socket_factory
        self.sock = None
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.thread = None
        self.armed = False
        self.request = ('STOP', 0.0, 0.0, 0.0)
        self.request_at = float('-inf')
        self.seq = 0
        self.send_error = None
        self.last_out = None

    def start(self):
        self.sock = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._send(('STOP', 0.0, 0.0, 0.0))
        self.done.clear()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def arm(self):
        # Local operator permission only. This is NOT a Pi/ESP32 acknowledgement.
        with self.lock:
            if self.send_error:
                return False
            self.armed = True
        return True

    def snapshot(self):
        with self.lock:
            return dict(connected=False, armed=self.armed,
                        reason='PI_UDP_UNACKNOWLEDGED', fault='NONE')

    def submit(self, decision, frame_timestamp):
        now = self.clock()
        cmd = decision.command
        move = decision.state in ('FOLLOWING', 'GESTURE_OVERRIDE')
        try:
            values = (float(cmd.vx_cm_s), float(cmd.vy_cm_s), float(cmd.w_rad_s))
            valid = all(math.isfinite(v) for v in values)
            # Reject impossible timing; never refresh a moving lease from old video.
            fresh = (math.isfinite(float(frame_timestamp))
                     and 0 <= now - float(frame_timestamp) <= self.stale_s)
        except (TypeError, ValueError, OverflowError):
            values, valid, fresh = (0.0, 0.0, 0.0), False, False
        with self.lock:
            if not (move and valid and fresh and self.armed and not self.send_error):
                self.request = ('STOP', 0.0, 0.0, 0.0)
                self.request_at = now
            else:
                vx, vy, w = values
                # Initial physical test: no backwards/lateral command until
                # orientation and rear-radar guard have been verified.
                if vx < 0 or abs(vy) > 0.001:
                    self.request = ('STOP', 0.0, 0.0, 0.0)
                else:
                    scale = min(1.0, self.MAX_LINEAR_CM_S / max(1e-9, math.hypot(vx, vy)))
                    self.request = ('RUN', vx * scale, vy * scale,
                                    max(-self.MAX_ANGULAR_RAD_S,
                                        min(self.MAX_ANGULAR_RAD_S, w)))
                self.request_at = now

    def _send(self, command):
        status, vx, vy, w = command
        if not self.sock:
            return
        self.seq += 1
        payload = dict(status=status, vx=float(vx), vy=float(vy), w=float(w), seq=self.seq)
        self.sock.sendto(json.dumps(payload, separators=(',', ':'), allow_nan=False).encode('utf-8'), self.dest)
        self.last_out = command

    def _loop(self):
        while not self.done.is_set():
            now = self.clock()
            with self.lock:
                command = (self.request if self.armed and now - self.request_at <= self.stale_s
                           else ('STOP', 0.0, 0.0, 0.0))
                if self.send_error:
                    command = ('STOP', 0.0, 0.0, 0.0)
            try:
                self._send(command)
            except OSError as exc:
                with self.lock:
                    self.send_error = type(exc).__name__
                    self.armed = False
                # Pi will timeout stale commands and force STOP.
                print('[PI UDP ERROR]', type(exc).__name__, exc)
            self.done.wait(self.PERIOD_S)

    def stop(self, reason='STOP'):
        with self.lock:
            self.request = ('STOP', 0.0, 0.0, 0.0)
            self.request_at = self.clock()
            if reason in ('EMERGENCY_STOP', 'RUNTIME_ERROR', 'EXIT'):
                self.armed = False
        try:
            self._send(('STOP', 0.0, 0.0, 0.0))
        except OSError:
            pass

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        self.stop('EXIT')
        for _ in range(2):
            try:
                self._send(('STOP', 0.0, 0.0, 0.0))
            except OSError:
                pass
        if self.sock:
            self.sock.close()
            self.sock = None
