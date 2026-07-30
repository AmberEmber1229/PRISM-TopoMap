import math
import threading


def _stamp_to_float(stamp):
    if stamp is None:
        return None
    if hasattr(stamp, 'to_sec'):
        return stamp.to_sec()
    return float(stamp)


def _format_value(value):
    if value is None:
        return 'none'
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        return '{:.6f}'.format(value)
    if isinstance(value, (list, tuple)):
        return '[' + ','.join(_format_value(item) for item in value) + ']'
    return str(value).replace(' ', '_')


class FlowTracer:
    """Small, dependency-free formatter for opt-in PRISM data-flow traces."""

    def __init__(self, enabled=False, every_n=1, descriptor_head_size=4,
                 registration_candidates=True):
        self.enabled = bool(enabled)
        self.every_n = max(1, int(every_n))
        self.descriptor_head_size = max(0, int(descriptor_head_size))
        self.registration_candidates = bool(registration_candidates)
        self._print_lock = threading.Lock()

    def sampled(self, frame):
        if not self.enabled:
            return False
        if frame is None:
            return True
        try:
            return int(frame) % self.every_n == 0
        except (TypeError, ValueError):
            return True

    def log(self, stage, frame=None, stamp=None, force=False, **fields):
        if not self.enabled:
            return
        if not force and not self.sampled(frame):
            return
        prefix = ['[FLOW]', '[ARCH=PYTHON]']
        if frame is not None:
            prefix.append('[FRAME={}]'.format(_format_value(frame)))
        stamp_float = _stamp_to_float(stamp)
        if stamp_float is not None:
            prefix.append('[STAMP={:.6f}]'.format(stamp_float))
        prefix.append('[STAGE={}]'.format(stage))
        details = ' '.join('{}={}'.format(key, _format_value(value))
                           for key, value in fields.items())
        line = ''.join(prefix)
        if details:
            line += ' ' + details
        with self._print_lock:
            print(line, flush=True)
