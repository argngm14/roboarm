import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "game"))

import arm as arm_lib  # noqa: E402
import hardware as hw  # noqa: E402

POSES = {
    "arm": {name: [2000, 2000, 2000, 2000] for name in
            ["safe", "b1_a", "b1_g", "c5_a", "c5_p"]},
    "grip": {"o1": 1500, "c1": 2500},
}


class RecordingBus(hw.DryBus):
    """Logs each arm wait: 'stop' (full stop) or 'blend' (carries straight on)."""
    def __init__(self):
        super().__init__()
        self.log = []

    def wait(self, names, timeout):
        self.log.append("stop")

    def wait_near(self, names, target, tol, timeout):
        self.log.append("blend")


def run_route(speed, blend):
    bus = RecordingBus()
    with patch.object(hw, "open_bus", return_value=bus), \
         patch.object(arm_lib, "load_poses", return_value=POSES), \
         patch.object(arm_lib.time, "sleep") as sleep:
        a = arm_lib.Arm(speed=speed, blend=blend)
        a.pick_and_place(1, 5)
    pauses = [c.args[0] for c in sleep.call_args_list]
    return bus.log, pauses


class BlendTests(unittest.TestCase):
    def test_game_py_blends_only_through_safe_and_pauses_at_aim(self):
        log, pauses = run_route("game", blend=True)
        # safe, b1_a, b1_g, b1_a, safe, c5_a, c5_p, c5_a, safe
        self.assertEqual(log, ["blend", "stop", "stop", "stop", "blend",
                               "stop", "stop", "stop", "stop"])
        self.assertEqual(pauses, [arm_lib.AIM_PAUSE, arm_lib.AIM_PAUSE])

    def test_no_blend_or_pause_outside_game_py(self):
        for speed, blend in (("game", False), ("test", True), ("test", False)):
            log, pauses = run_route(speed, blend)
            self.assertEqual(set(log), {"stop"}, (speed, blend))
            self.assertEqual(pauses, [], (speed, blend))


class FakeTerm:
    def __init__(self, keys):
        self.keys = list(keys)

    def read_key(self, timeout=0.0):
        return self.keys.pop(0) if self.keys else None


def press(keys):
    """Feed keys the way a wait loop does. Returns True if any key meant stop."""
    term = FakeTerm(keys)
    stopped = False
    while term.keys:
        stopped |= hw._stop_key(term, term.read_key())
    return stopped


class TypeAheadTests(unittest.TestCase):
    def setUp(self):
        hw.typeahead.clear()
        hw.typeahead.enabled = True
        self.out = patch.object(hw.sys, "stdout")
        self.out.start()

    def tearDown(self):
        self.out.stop()
        hw.typeahead.enabled = False
        hw.typeahead.clear()

    def test_enter_queues_one_command_only(self):
        press("15\r")
        press("k8\r")                       # queue full: ignored
        self.assertEqual(hw.typeahead.take(), ("15", ""))

    def test_esc_cancels_queue_and_typed_text(self):
        press("15\r2")
        press("\x1b")
        self.assertEqual(hw.typeahead.take(), (None, ""))

    def test_arrow_key_is_not_esc(self):
        press("15\r")
        press("\x1b[a")                     # up arrow
        self.assertEqual(hw.typeahead.take(), ("15", ""))

    def test_space_stops_and_is_not_typed(self):
        self.assertTrue(press("1 "))
        self.assertEqual(hw.typeahead.take(), (None, "1"))

    def test_backspace_and_half_typed_text(self):
        press("k9\x7f8")
        self.assertEqual(hw.typeahead.take(), (None, "k8"))

    def test_disabled_outside_game_py(self):
        hw.typeahead.enabled = False
        press("15\r")
        self.assertEqual(hw.typeahead.take(), (None, ""))


if __name__ == "__main__":
    unittest.main()
