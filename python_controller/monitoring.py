"""Evidence formatting and recording. Missing measurements stay null."""
import json
import math
from datetime import datetime, timezone
from pathlib import Path
import time
import queue
import threading


def vector(data, key, integer=False):
    value = (data or {}).get(key)
    if not isinstance(value,list) or len(value)!=3:
        return None
    if not all(type(v) in (int,float) and math.isfinite(v) and (not integer or type(v) is int) for v in value):
        return None
    return value.copy()


def motor_feedback(telemetry, radius_cm):
    target,measured = vector(telemetry,'wheel_target'),vector(telemetry,'wheel_speed')
    factor = 60/(2*math.pi*radius_cm)
    return dict(target_cm_s=target,measured_cm_s=measured,
                target_rpm=[v*factor for v in target] if target is not None else None,
                measured_rpm=[v*factor for v in measured] if measured is not None else None,
                pwm=vector(telemetry,'wheel_pwm'),encoder_count=vector(telemetry,'encoder_count',True))


class TestBook:
    NAMES = ('Camera','Target tracking','Four gestures','Follow command directions','ESP32 communication','Lifted-wheel motors','Controlled floor drive')
    def __init__(self):
        self.rows = [dict(test=name,result='NOT TESTED',notes='',recorded_at=None,mode=None,evidence_source=None) for name in self.NAMES]

    def record(self,index,result,notes,mode='SIMULATION',source='MANUAL_OPERATOR_RECORD'):
        if result not in ('PASS','FAIL','NOT TESTED') or not 0<=index<len(self.rows):
            raise ValueError('Invalid test result')
        if result != 'NOT TESTED' and not notes.strip():
            raise ValueError('A result requires observations/evidence in notes')
        self.rows[index].update(result=result,notes=notes,mode=mode,evidence_source=source,
                                recorded_at=datetime.now().astimezone().isoformat())


class SessionLogger:
    def __init__(self,folder,mode,source):
        folder = Path(folder)
        self.directory = folder/datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True)
        self.path = self.directory/'events.jsonl'
        self.file = self.path.open('w',encoding='utf-8')
        self.mode,self.source = mode,source
        self.last_write = -math.inf
        self.queue=queue.Queue(maxsize=256)
        self.error=None
        self.worker=threading.Thread(target=self._writer,name='session-jsonl-writer',daemon=True)
        self.worker.start()

    def _writer(self):
        try:
            while True:
                entry=self.queue.get()
                if entry is None: break
                self.file.write(json.dumps(entry,ensure_ascii=True,allow_nan=False)+'\n')
                self.file.flush()
        except Exception as exc:
            self.error=type(exc).__name__
        finally:
            self.file.close()

    def write(self,record,force=False):
        now=time.monotonic()
        if self.error: raise OSError('Log writer failed: '+self.error)
        if not force and now-self.last_write < .1: return
        entry = {**record,'timestamp':datetime.now().astimezone().isoformat(),'monotonic_s':now,
                 'program_mode':self.mode,'evidence_source':self.source}
        self.queue.put_nowait(entry)
        if not force:self.last_write=now

    def save_tests(self,book):
        path=self.directory/'test_results.json'
        path.write_text(json.dumps(dict(program_mode=self.mode,evidence_source=self.source,
                                       operator_recorded=True,tests=book.rows),indent=2)+'\n',encoding='utf-8')
        return path

    def close(self):
        self.queue.put(None,timeout=1)
        self.worker.join(timeout=2)
        if self.worker.is_alive(): raise OSError('Log writer did not close')
        if self.error: raise OSError('Log writer failed: '+self.error)
