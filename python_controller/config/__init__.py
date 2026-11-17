import copy
import json
import math
from pathlib import Path


def load_config(path=None):
    base = Path(__file__).with_name('default.json')
    config = json.loads(base.read_text(encoding='utf-8'))
    schema = copy.deepcopy(config)
    if path:
        custom = json.loads(Path(path).read_text(encoding='utf-8'))
        for section, values in custom.items():
            if section not in config or not isinstance(values, dict):
                raise ValueError(f'Invalid config section: {section}')
            unknown = values.keys() - config[section].keys()
            if unknown:
                raise ValueError(f'Unknown config keys: {sorted(unknown)}')
            config[section].update(values)
    for section, values in config.items():
        for key, value in values.items():
            expected = schema[section][key]
            if isinstance(expected,bool):
                valid_type = type(value) is bool
            elif isinstance(expected,(int,float)):
                valid_type = isinstance(value,(int,float)) and not isinstance(value,bool)
            else:
                valid_type = isinstance(value,type(expected))
            if not valid_type:
                raise ValueError(f'Invalid config type: {section}.{key}')
            if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
                raise ValueError(f'Non-finite config: {section}.{key}')
    f, s, g, c = (config[k] for k in ('follow', 'safety', 'gesture', 'communication'))
    for key, ceiling in [('max_linear_cm_s', 5), ('max_angular_rad_s', .5), ('linear_accel_cm_s2',25), ('angular_accel_rad_s2',2)]:
        if not 0 < f[key] <= ceiling:
            raise ValueError(f'Unsafe follow setting: {key}')
    if not .05 <= s['perception_timeout_s'] <= .5 or not .2 <= s['telemetry_timeout_s'] <= 1:
        raise ValueError('Unsafe freshness timeout')
    if not 0 < f['size_min'] <= f['desired_height_ratio'] <= f['size_max'] < 1:
        raise ValueError('Invalid relative size range')
    if not 0 < f['filter_alpha'] <= 1 or f['forward_sign'] not in [-1, 1] or f['turn_sign'] not in [-1, 1]:
        raise ValueError('Invalid filter or sign')
    for key in ('horizontal_deadband','height_deadband','size_jump_ratio','center_jump_ratio','overlap_iou','distance_step'):
        if not 0 < f[key] < 1:
            raise ValueError(f'Invalid ratio: {key}')
    if not 0 < f['turn_gain'] <= 5 or not 0 < f['distance_gain'] <= 100:
        raise ValueError('Invalid control gain')
    if not 0 < g['hold_s'] <= 2 or not 2 <= g['min_frames'] <= 30 or not .5 <= g['confidence'] <= 1:
        raise ValueError('Invalid gesture confirmation')
    if not .02 <= c['send_period_s'] <= .1 or c['command_port'] != 8888 or c['telemetry_port'] != 8889:
        raise ValueError('Protocol ports/timing must match audited firmware')
    if not 0 < c['wheel_radius_cm'] <= 100:
        raise ValueError('Invalid wheel radius for telemetry RPM conversion')
    m=config['motor_test']
    if (not 0<m['linear_cm_s']<=5 or not 0<m['angular_rad_s']<=.5 or
        not .1<=m['duration_s']<=m['max_duration_s']<=2 or not .08<=m['send_period_s']<=.12 or
        not .1<=m['ui_lease_s']<=.25 or not .2<=m['telemetry_timeout_s']<=.7 or
        not .5<=m['stop_feedback_timeout_s']<=3 or not 0<m['body_radius_cm']<=100 or
        type(m['min_encoder_delta']) is not int or m['min_encoder_delta']<1 or not 0<m['min_wheel_speed_cm_s']<=1):
        raise ValueError('Unsafe motor test settings')
    if set(g['mapping'].values()) - {'RESUME','PAUSE','CLOSER','FARTHER'}:
        raise ValueError('Invalid gesture action')
    if not 0 < g['max_gap_s'] <= s['perception_timeout_s'] or not .1 <= g['release_s'] <= 2:
        raise ValueError('Invalid gesture continuity timing')
    if not .1 <= s['person_confidence'] <= 1 or not .1 <= config['vision']['confidence'] <= 1:
        raise ValueError('Invalid person confidence')
    for section,key in [('camera','index'),('camera','width'),('camera','height'),('vision','image_size'),('vision','max_hands'),('gesture','min_frames')]:
        if type(config[section][key]) is not int or config[section][key] < (0 if key=='index' else 1):
            raise ValueError(f'Invalid integer config: {section}.{key}')
    return copy.deepcopy(config)
