import argparse
import json
from pathlib import Path
import time

from python_controller.config import load_config
from python_controller.control.state_machine import Controller
from python_controller.models import PersonObservation, PerceptionFrame, RadarObservation


ROOT = Path(__file__).resolve().parents[1]


def demo(config):
    """Deterministic synthetic observations. This does not test camera or ML quality."""
    c = Controller(config)
    c.select_target(7); c.confirm_target()
    scenarios = [('ARM',True,None),('RESUME',True,None),(None,False,None),
                 (None,True,None),('RESUME',True,None),(None,True,True),
                 (None,True,None),('RESUME',True,None),('ESTOP',True,None)]
    for i,(action,visible,obstacle) in enumerate(scenarios):
        now = 1+i*.1
        people = (PersonObservation(7,(240,100,400,300),.95,now),) if visible else ()
        f = PerceptionFrame(now,640,480,people)
        radar = RadarObservation(True,.4,now,True) if obstacle else None
        d = c.step(f,now,action,radar=radar)
        print(json.dumps(dict(mode='SIMULATION',synthetic=True,state=d.state,reason=d.reason,
                              vx=d.command.vx_cm_s,vy=d.command.vy_cm_s,w=d.command.w_rad_s)))
    return 0


def run(config, hardware=False):
    import cv2
    from python_controller.vision.camera import Camera
    from python_controller.vision.person_detector import PersonDetector
    from python_controller.vision.person_tracker import TargetSelector
    from python_controller.vision.gesture_recognizer import GestureRecognizer
    from python_controller.visualization.debug_view import DebugView
    from python_controller.communication.esp32_udp import UdpClient
    c = Controller(config,hardware)
    camera = gestures = view = client = None
    try:
        # Models load before connecting to the motor controller.
        detector = PersonDetector(config['vision'])
        gestures = GestureRecognizer(config['vision'],config['gesture'])
        camera = Camera(config['camera'])
        if hardware:
            client = UdpClient(config)
            client.start()  # Repeated STOP only; no motion before A then R / Open Palm.
        view = DebugView()
        last_display = time.monotonic()
        pending = None
        while True:
            image, timestamp = camera.read()
            people = detector.detect(image,timestamp)
            candidate = TargetSelector.candidate(people)
            if c.target_id is None and candidate is not None:
                c.select_target(candidate)
            gesture,hands = gestures.recognize(image,people,c.target_id,timestamp)
            now = time.monotonic()
            snapshot = client.snapshot() if client else dict(connected=False,armed=False,reason='SIMULATION_NO_UDP',fault='NONE')
            frame = PerceptionFrame(timestamp,image.shape[1],image.shape[0],people,gesture)
            action = pending
            pending = None
            # Mouse events and keys are processed before deciding the next command.
            for event in view.drain_events():
                if event[0] == 'ESTOP':
                    action = 'ESTOP'
                elif action != 'ESTOP' and event[0] == 'SELECT':
                    chosen = TargetSelector.at_point(people,event[1],event[2])
                    if chosen is not None:
                        c.select_target(chosen)
            if action == 'CONFIRM':
                c.confirm_target(); action = None
            elif action == 'CLEAR':
                c.select_target(None); action = 'PAUSE'
            if action == 'ARM' and client:
                if not client.arm():
                    print('ARM refused: fresh matching STOPPED telemetry required')
                    action = None
                snapshot = client.snapshot()
            if action == 'ESTOP' and client:
                client.stop('EMERGENCY_STOP')
            communication_ok = not hardware or (snapshot['connected'] and (not c.armed or snapshot['armed']))
            decision = c.step(frame,now,action,communication_ok,
                              snapshot['fault'] if hardware else 'NONE')
            if client:
                client.submit(decision,timestamp)
            fps = 1/max(.000001,now-last_display)
            last_display = now
            canvas = view.draw(image,people,hands,decision,gesture,c.armed,c.follow.desired,
                               snapshot,fps,(now-timestamp)*1000,candidate,'HARDWARE' if hardware else 'SIMULATION')
            view.show(canvas)
            key = cv2.waitKey(1)&0xff
            if key in (27,ord('q')) or view.closed():
                if client: client.stop('EXIT')
                break
            pending = {ord(' '):'ESTOP',ord('e'):'ESTOP',ord('p'):'PAUSE',ord('a'):'ARM',
                       ord('r'):'RESUME',13:'CONFIRM',10:'CONFIRM',ord('c'):'CLEAR'}.get(key)
            if pending in ('ESTOP','PAUSE') and client:
                client.stop(pending)
    except KeyboardInterrupt:
        print('STOP: keyboard interrupt')
    except Exception as exc:
        if client: client.stop('RUNTIME_ERROR')
        print(f'STOP: {type(exc).__name__}: {exc}')
        return 1
    finally:
        # Stop the transport before releasing any potentially blocking vision resources.
        if client: client.close()
        if camera: camera.close()
        if gestures: gestures.close()
        if view: view.close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Simulation-first person following and gesture control')
    parser.add_argument('--config',help='JSON overrides; defaults are always merged')
    parser.add_argument('--mode',choices=('SIMULATION','HARDWARE'),default='SIMULATION')
    parser.add_argument('--demo',action='store_true',help='Synthetic, headless, dependency-free control demo; never sends UDP')
    parser.add_argument('--ui',choices=('opencv','dashboard'),default='opencv',help='Dashboard is the default in run_simulation.bat; OpenCV retained for compatibility')
    parser.add_argument('--dashboard-preview',action='store_true',help='Clearly labeled synthetic dashboard; no hardware allowed')
    parser.add_argument('--smoke-seconds',type=float,help='Close dashboard after this many seconds (0.1 to 60)')
    parser.add_argument('--esp32-host',help='Numeric ESP32 IPv4 address; never the laptop IP')
    parser.add_argument('--acknowledge-hardware-risk',action='store_true',help='Explicit hardware opt-in; physical sign verification also required')
    args = parser.parse_args(argv)
    if args.dashboard_preview and (args.mode=='HARDWARE' or args.ui!='dashboard'):
        parser.error('Synthetic preview requires --ui dashboard and SIMULATION')
    if args.smoke_seconds is not None and (not .1<=args.smoke_seconds<=60 or args.ui!='dashboard'):
        parser.error('--smoke-seconds requires dashboard and a finite duration from 0.1 to 60')
    if args.mode == 'HARDWARE' and (args.demo or not args.acknowledge_hardware_risk or not args.esp32_host):
        parser.error('HARDWARE requires --esp32-host and --acknowledge-hardware-risk; --demo cannot use HARDWARE')
    if args.mode=='HARDWARE' and args.ui!='dashboard':
        parser.error('Hardware is restricted to dashboard MOTOR TEST; automatic hardware following is disabled')
    try:
        config = load_config(args.config)
        if args.esp32_host: config['communication']['host'] = args.esp32_host
        for key in ('model_path','gesture_model_path'):
            p = Path(config['vision'][key])
            if not p.is_absolute(): config['vision'][key] = str(ROOT/p)
        if args.demo: return demo(config)
        if args.ui=='dashboard':
            from python_controller.visualization.dashboard import run_dashboard
            return run_dashboard(config,args.mode=='HARDWARE',args.dashboard_preview,args.smoke_seconds)
        return run(config,args.mode=='HARDWARE')
    except (ValueError,OSError,ImportError) as exc:
        print(f'STOP: {exc}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
