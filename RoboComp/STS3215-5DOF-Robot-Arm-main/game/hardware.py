"""
hardware.py - the only file that talks to the motors.

It uses the course code (arm_helpers.py, arm_settings.py, scservo_sdk/)
from the STS3215-5DOF-Robot-Arm folder. Put this "game" folder INSIDE that
folder:

    STS3215-5DOF-Robot-Arm/
        arm_helpers.py, arm_settings.py, scservo_sdk/, scripts/, ...
        game/
            hardware.py   arm.py   teach.py   poses.json

Serial port, servo IDs and baud rate still come from arm_settings.py.
"""

import sys
import time
from pathlib import Path

# Practice mode: True = never touch the motors, just run the logic.
# Leave True while practising on your laptop.
# Set to False on the Raspberry Pi.
DRY_RUN = False

ARM_JOINTS = ("base_yaw", "shoulder", "elbow", "wrist")
GRIPPER = "gripper"
ALL_JOINTS = ARM_JOINTS + (GRIPPER,)

# ---------------------------------------------------------------- JOINT LIMITS
# Allowed range for each motor, in raw ticks (min, max).
#
# 0-4095 is the servo's full turn, but the arm and gripper hit their mechanical
# stops well before that. Record each joint's real range and enter it here.
# Leave a small margin (about 30-50 ticks) inside each hard stop, so the motor
# never ends up pushing against it.
#
# How limits are used:
#   - teach mode: a set/nudge past a limit stops AT the limit (and tells you)
#   - everything else (go, pick, place, run, arm.py): a move with any target
#     outside the limits is refused before anything moves
#   - saving a pose outside the limits gives a warning
JOINT_LIMITS = {
    "base_yaw": (500, 3500),   # motor 1: right (R) 500 ... left (L) 3500
    "shoulder": (1300, 2800),  # motor 2: forward (F) 1300 ... back (B) 2800
                               # NOTE: shoulder range depends on elbow position (to be adjusted)
    "elbow":    (1000, 3000),  # motor 3: down (D) 1000 ... up (U) 3000
    "wrist":    (1000, 3000),  # motor 4: down (D) 1000 ... up (U) 3000
    "gripper":  (1300, 3000),  # motor 5: open (O) 1300 ... close (C) 3000
}

for _name, (_lo, _hi) in JOINT_LIMITS.items():
    if not 0 <= _lo < _hi <= 4095:
        raise ValueError(f"JOINT_LIMITS['{_name}'] = ({_lo}, {_hi}) must satisfy 0 <= min < max <= 4095")


def outside_limits(targets):
    """List of (joint, value, min, max) for every target outside its limit."""
    bad = []
    for name, value in targets.items():
        lo, hi = JOINT_LIMITS[name]
        if not lo <= value <= hi:
            bad.append((name, value, lo, hi))
    return bad


def clamp(name, value):
    lo, hi = JOINT_LIMITS[name]
    return max(lo, min(hi, value))


def describe(bad):
    return ", ".join(f"{n} {v} (limit {lo}-{hi})" for n, v, lo, hi in bad)

# ------------------------------------------------------------------------------

# Pretend starting position, used only in practice mode.
DRY_START = {name: 2048 for name in ALL_JOINTS}

POLL_SECONDS = 0.02


class StopPressed(Exception):
    """Space was pressed during a move. The arm stopped and is holding still."""


def _say(msg):
    # The terminal is in raw mode during a move, so "\n" alone won't return the cursor.
    sys.stdout.write(f"\r\n  {msg}\r\n")
    sys.stdout.flush()


class TypeAhead:
    """
    game.py only: type the next command while the arm moves. Enter queues it
    (one command at most) and it runs when the current one finishes. Esc cancels
    the queued command. Space still stops the arm, so type "1 5" as "15".
    """
    def __init__(self):
        self.enabled = False
        self.text = ""        # typed so far, not yet Enter'd
        self.queued = None

    def feed(self, key):
        if not self.enabled:
            return
        if key in ("\r", "\n"):
            line, self.text = self.text.strip(), ""
            if not line:
                return
            if self.queued:
                _say(f"queue full: '{self.queued}' is next (Esc cancels it)")
            else:
                self.queued = line
                _say(f"queued: {line}   (Esc = cancel)")
        elif key == "\x1b":
            if self.queued or self.text:
                _say("queue cancelled")
            self.clear()
        elif key in ("\x7f", "\x08"):
            self.text = self.text[:-1]
        elif key.isprintable():
            self.text += key

    def clear(self):
        self.text, self.queued = "", None

    def take(self):
        """The queued command (or None), and any half-typed text. Both are cleared."""
        out = (self.queued, self.text)
        self.clear()
        return out


typeahead = TypeAhead()


def _stop_key(term, key):
    """True for Space / Ctrl+C (stop). Any other key goes to the type-ahead queue."""
    if key in (" ", "\x03"):
        return True
    if key == "\x1b":
        nxt = term.read_key(0.01)
        if nxt in ("[", "o"):          # arrow or function key, not Esc: ignore the whole sequence
            for _ in range(4):
                k = term.read_key(0.01)
                if not k or k.isalpha() or k == "~":
                    break
            return False
        typeahead.feed("\x1b")
        return _stop_key(term, nxt) if nxt else False
    if key:
        typeahead.feed(key)
    return False


# ---------------------------------------------------------------- real arm

class LiveBus:
    def __init__(self):
        repo = Path(__file__).resolve().parents[1]
        if not (repo / "arm_helpers.py").exists():
            raise RuntimeError(
                f"Can't find arm_helpers.py in {repo}. Put the 'game' folder "
                "inside the STS3215-5DOF-Robot-Arm folder."
            )
        if str(repo) not in sys.path:
            sys.path.insert(0, str(repo))

        import arm_helpers
        import arm_settings
        from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE

        self.h = arm_helpers
        self.s = arm_settings
        self.COMM_SUCCESS = COMM_SUCCESS
        self.TORQUE_ADDR = SMS_STS_TORQUE_ENABLE

        missing = [n for n in ALL_JOINTS if n not in arm_settings.SERVO_IDS]
        if missing:
            raise RuntimeError(f"arm_settings.SERVO_IDS is missing: {missing}")

        try:
            from terminal_keys import RawTerminal
            self.RawTerminal = RawTerminal
        except ImportError:
            self.RawTerminal = None

        self.port, self.ph = arm_helpers.connect()

    def _id(self, name):
        return self.s.SERVO_IDS[name]

    def read(self, names=ALL_JOINTS):
        """Actual positions reported by the servos (raw ticks 0-4095)."""
        return {n: self.h.read_servo(self.ph, self._id(n)) for n in names}

    def _drain(self):
        """Pause briefly and throw away stray bytes (e.g. left over from the
        previous move packet), so the next reply is not garbled."""
        time.sleep(0.02)
        try:
            self.port.ser.reset_input_buffer()
        except Exception:
            pass

    def torque(self, names, on):
        # Torque on/off is safe to repeat, so retry if the reply is garbled.
        for n in names:
            for _attempt in range(3):
                self._drain()
                result, error = self.ph.write1ByteTxRx(self._id(n), self.TORQUE_ADDR, 1 if on else 0)
                if result == self.COMM_SUCCESS:
                    break
            self.h.check_result(self.ph, result, error, f"Torque {'on' if on else 'off'} for {n}")

    def send(self, targets, speeds, accs):
        """One synchronized packet. Each joint can have its own speed/acceleration."""
        try:
            for n, pos in targets.items():
                pos = int(pos)
                if not 0 <= pos <= 4095:
                    raise ValueError(f"{n} position {pos} is outside 0-4095")
                if not self.ph.SyncWritePosEx(self._id(n), pos, int(speeds[n]), int(accs[n])):
                    raise RuntimeError(f"Could not add {n} to synchronized move")
            result = self.ph.groupSyncWrite.txPacket()
            if result != self.COMM_SUCCESS:
                raise RuntimeError(self.ph.getTxRxResult(result))
        finally:
            self.ph.groupSyncWrite.clearParam()

    def hold(self, names=ALL_JOINTS):
        """Freeze where the arm is now: target = current position, torque on."""
        pos = self.read(names)
        self.send(pos, {n: 1000 for n in names}, {n: 50 for n in names})
        self.torque(names, True)
        return pos

    def _any_moving(self, names):
        for n in names:
            moving, result, error = self.ph.ReadMoving(self._id(n))
            self.h.check_result(self.ph, result, error, f"Read moving status for {n}")
            if moving:
                return True
        return False

    def wait(self, names, timeout):
        """Wait for the move to finish. Space (or Ctrl+C) stops and holds."""
        start = time.monotonic()
        time.sleep(POLL_SECONDS)
        keys = _NoKeys()
        if self.RawTerminal is not None and sys.stdin.isatty():
            keys = self.RawTerminal()
        with keys as term:
            while True:
                if _stop_key(term, term.read_key(POLL_SECONDS)):
                    self.hold(names)   # only the moving joints; gripper keeps its squeeze
                    raise StopPressed()
                if not self._any_moving(names):
                    return
                if time.monotonic() - start > timeout:
                    self.hold(names)
                    raise RuntimeError(
                        "Move took too long, so the arm may be blocked. Stopped and holding."
                    )

    def wait_near(self, names, target, tol, timeout):
        """
        Wait only until every joint is within `tol` ticks of `target`, not until it
        has stopped, so the next move can carry straight on. Space still stops and holds.
        A garbled reply is skipped (the arm is moving fast); several in a row stop the arm.
        """
        start = time.monotonic()
        keys = _NoKeys()
        if self.RawTerminal is not None and sys.stdin.isatty():
            keys = self.RawTerminal()
        bad_reads = 0
        with keys as term:
            while True:
                if _stop_key(term, term.read_key(0.005)):
                    self.hold(names)
                    raise StopPressed()
                try:
                    pos = self.read(names)
                    bad_reads = 0
                except RuntimeError:
                    bad_reads += 1
                    if bad_reads >= 5:
                        self.hold(names)
                        raise
                    continue
                if max(abs(pos[n] - target[n]) for n in names) <= tol:
                    return
                if time.monotonic() - start > timeout:
                    self.hold(names)
                    raise RuntimeError(
                        "Move took too long, so the arm may be blocked. Stopped and holding."
                    )

    def wait_settled(self, name, min_time, max_time=2.0, still_ticks=3):
        """
        Wait until one motor stops moving. Used for the gripper: when it squeezes
        a block it stops before reaching its target, so the servo's own 'moving'
        flag can't be trusted, but its position stops changing either way.
        """
        start = time.monotonic()
        last = self.read([name])[name]
        still = 0
        while True:
            time.sleep(0.03)
            pos = self.read([name])[name]
            still = still + 1 if abs(pos - last) <= still_ticks else 0
            last = pos
            elapsed = time.monotonic() - start
            if (still >= 3 and elapsed >= min_time) or elapsed >= max_time:
                return

    def close(self):
        self.port.closePort()


class _NoKeys:
    """Used when there's no terminal to read Space from."""
    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read_key(self, timeout):
        time.sleep(timeout)
        return None


# ---------------------------------------------------------------- practice mode

class DryBus:
    def __init__(self):
        self.pos = dict(DRY_START)

    def read(self, names=ALL_JOINTS):
        return {n: self.pos[n] for n in names}

    def torque(self, names, on):
        pass

    def send(self, targets, speeds, accs):
        for n, p in targets.items():
            if not 0 <= int(p) <= 4095:
                raise ValueError(f"{n} position {p} is outside 0-4095")
        self.pos.update({n: int(p) for n, p in targets.items()})

    def hold(self, names=ALL_JOINTS):
        return self.read(names)

    def wait(self, names, timeout):
        pass

    def wait_near(self, names, target, tol, timeout):
        pass

    def wait_settled(self, name, min_time, max_time=2.0, still_ticks=3):
        pass

    def close(self):
        pass


def open_bus():
    return DryBus() if DRY_RUN else LiveBus()
